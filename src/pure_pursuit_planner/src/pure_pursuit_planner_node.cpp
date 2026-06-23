// Modified for PCT integration:
//   - Path topic: /pct_path (was tgt_path)
//   - Odometry topic: /Odometry_open3d (was odom), frame_id=map, child=base_link
//   - ck (curvature) computed from path geometry (PCT stores z-height, not curvature)
//   - cyaw (tangent heading) computed from adjacent waypoints (PCT publishes identity quaternion)
//   - Path updates: old path_subscribe_flag removed → new path always replaces old

#include "pure_pursuit_planner/pure_pursuit_planner_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist.hpp"


namespace pure_pursuit_planner {

// ── helper: compute tangent yaw from consecutive points ──────────────────
static std::vector<double> compute_yaw_from_path(const std::vector<double>& cx,
                                                  const std::vector<double>& cy) {
    std::vector<double> cyaw(cx.size(), 0.0);
    if (cx.size() < 2) return cyaw;

    for (size_t i = 0; i < cx.size() - 1; i++) {
        cyaw[i] = std::atan2(cy[i + 1] - cy[i], cx[i + 1] - cx[i]);
    }
    // last point inherits previous yaw
    cyaw.back() = cyaw[cyaw.size() - 2];
    return cyaw;
}

// ── helper: compute discrete curvature from path geometry ────────────────
// κ_i = Δθ / Δs  where Δθ is turning angle at point i, Δs is avg segment length
static std::vector<double> compute_curvature_from_path(const std::vector<double>& cx,
                                                        const std::vector<double>& cy) {
    std::vector<double> ck(cx.size(), 0.0);
    if (cx.size() < 3) return ck;

    for (size_t i = 1; i < cx.size() - 1; i++) {
        // vectors: prev→cur and cur→next
        double dx1 = cx[i] - cx[i - 1];
        double dy1 = cy[i] - cy[i - 1];
        double dx2 = cx[i + 1] - cx[i];
        double dy2 = cy[i + 1] - cy[i];

        double len1 = std::hypot(dx1, dy1);
        double len2 = std::hypot(dx2, dy2);
        if (len1 < 1e-6 || len2 < 1e-6) continue;

        // turning angle Δθ
        double dot = (dx1 * dx2 + dy1 * dy2) / (len1 * len2);
        dot = std::clamp(dot, -1.0, 1.0);
        double dtheta = std::acos(dot);

        // arc length ≈ average of two segments
        double ds = 0.5 * (len1 + len2);

        ck[i] = dtheta / ds;  // curvature [rad/m]
    }
    // endpoints inherit from neighbor
    ck[0] = ck[1];
    ck.back() = ck[ck.size() - 2];
    return ck;
}


PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
: Node("pure_pursuit_node", options), planner_(config_) {

    declareAndGetParameters();

    // PCT: subscribe to /pct_path (nav_msgs/Path, frame_id=map)
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/pct_path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    // open3d_loc: subscribe to /Odometry_open3d (frame_id=map, child_frame_id=base_link)
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry_open3d", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PurePursuitNode::timerCallback, this));
    planner_ = PurePursuitComponent(config_);  // re-init with populated config_
}

void PurePursuitNode::declareAndGetParameters() {
    config_.k = this->declare_parameter("k", 0.5);
    config_.Lfc = this->declare_parameter("Lfc", 0.8);
    config_.Kp = this->declare_parameter("Kp", 1.0);
    config_.dt = this->declare_parameter("dt", 0.1);
    config_.goal_threshold = this->declare_parameter("goal_threshold", 0.4);
    config_.max_acceleration = this->declare_parameter("max_acceleration", 0.08);
    config_.minCurvature = this->declare_parameter("minCurvature", 0.0);
    config_.maxCurvature = this->declare_parameter("maxCurvature", 3.0);
    config_.minVelocity = this->declare_parameter("minVelocity", 0.1);
    config_.maxVelocity = this->declare_parameter("maxVelocity", 0.5);
    config_.maxAngularVelocity = this->declare_parameter("maxAngularVelocity", 0.5);
    config_.obstacle_th = this->declare_parameter("obstacle_th", 0.5);
}

void PurePursuitNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty /pct_path — ignoring.");
        return;
    }

    // Verify frame — PCT publishes in 'map', Odometry_open3d is also in 'map'
    if (msg->header.frame_id != "map" && msg->header.frame_id != "odom") {
        RCLCPP_WARN(this->get_logger(),
            "Path frame_id '%s' is not 'map' or 'odom'. Behavior may be incorrect.",
            msg->header.frame_id.c_str());
    }

    // Clear old path and rebuild
    cx_.clear(); cy_.clear(); cyaw_.clear(); ck_.clear();

    // Step 1: extract raw x/y from PCT path
    std::vector<double> raw_x, raw_y;
    for (const auto& pose : msg->poses) {
        raw_x.push_back(pose.pose.position.x);
        raw_y.push_back(pose.pose.position.y);
    }

    // Step 2: compute cyaw (tangent heading) from adjacent points
    //         PCT publishes identity quaternion, so we compute correct heading
    cyaw_ = compute_yaw_from_path(raw_x, raw_y);

    // Step 3: compute ck (curvature) from path geometry
    //         PCT stores z-height in position.z, NOT curvature
    ck_ = compute_curvature_from_path(raw_x, raw_y);

    // Step 4: store final path
    cx_ = raw_x;
    cy_ = raw_y;

    // Reset state for new path
    path_received_ = true;
    planner_.odom_sub_flag = false;   // trigger re-init of nearest-point search
    planner_.oldNearestPointIndex = -1;

    RCLCPP_INFO(this->get_logger(),
        "Received /pct_path: %zu waypoints, frame=%s. "
        "Computed yaw and curvature from geometry.",
        cx_.size(), msg->header.frame_id.c_str());
}

void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Odometry_open3d: header.frame_id = 'map', child_frame_id = 'base_link'
    // Position is directly in map frame → matches /pct_path coordinates
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_vx_ = msg->twist.twist.linear.x;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll_tmp, pitch_tmp, yaw_tmp;
    mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

    current_pose_.yaw = yaw_tmp;

    pose_received_ = true;
}

void PurePursuitNode::timerCallback() {
    if (!path_received_ || !pose_received_) return;

    auto cmd_velocity = planner_.computeVelocity(cx_, cy_, cyaw_, ck_, current_pose_, current_vx_);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = cmd_velocity[0];
    cmd_vel.angular.z = cmd_velocity[1];

    cmd_vel_pub_->publish(cmd_vel);
}

}  // namespace pure_pursuit_planner
