#include "art_planner_ros/planner_ros.h"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include <art_planner/planners/prm_motion_cost.h> // TODO: This include is kind of ugly.
#include <art_planner/objectives/motion_cost_objective.h> // TODO: This include is kind of ugly.
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <art_planner_ros/utils.h>



using namespace art_planner;
using namespace std::chrono_literals;



void PlannerRos::mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr map_msg) {
  std::lock_guard<std::mutex> lock(map_queue_mutex_);

  if (!map_queue_.map) map_queue_.map.reset(new grid_map::GridMap());
  grid_map::GridMapRosConverter::fromMessage(*map_msg, *map_queue_.map);
  map_queue_.map->convertToDefaultStartIndex();

  map_queue_.header = map_msg->header;
  map_queue_.info = map_msg->info;
}



void PlannerRos::stopPlanningContinuously() {
  planning_continuously_ = false;
  std::lock_guard<std::mutex> lock(planning_thread_mutex_);
  if (continuous_planning_thread_.joinable()) {
    continuous_planning_thread_.join();
  }
}



void PlannerRos::planContinuouslyThread() {
  rclcpp::Time last_plan_start;
  const rclcpp::Duration replan_time = rclcpp::Duration::from_seconds(1.0/params_->planner.replan_freq);

  while (planning_continuously_) {
    last_plan_start = node_->now();

    planFromCurrentRobotPose();

    const auto cur_time = node_->now();
    const auto sleep_time = replan_time - (cur_time - last_plan_start);
    if (sleep_time.seconds() > 0.0) {
      // Sleep until we want to replan.
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time.seconds()));
    }
  }
}



bool PlannerRos::getCurrentRobotPose(geometry_msgs::msg::PoseStamped* pose) const {
  // Get robot pose.
  geometry_msgs::msg::TransformStamped pose_tf;
  try {
    pose_tf = tf_buffer_.lookupTransform(map_queue_.header.frame_id,
                                    params_->robot.base_frame,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(0.1));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(node_->get_logger(), "Could not get robot pose from TF. Cannot plan!");
    publishFeedback(Feedback::NO_ROBOT_TF);
    return false;
  }

  tf2::Stamped<tf2::Transform> tf_tf2;
  tf2::fromMsg(pose_tf, tf_tf2);

  // Get feet center pose from base.
  tf2::Transform offset;
  offset.setIdentity();
  offset.setOrigin(tf2::Vector3(0, 0, params_->robot.feet.offset.z));

  tf_tf2 *= offset;

  pose->header = pose_tf.header;
  pose->pose.position.x = tf_tf2.getOrigin().x();
  pose->pose.position.y = tf_tf2.getOrigin().y();
  pose->pose.position.z = tf_tf2.getOrigin().z();
  pose->pose.orientation.x = tf_tf2.getRotation().x();
  pose->pose.orientation.y = tf_tf2.getRotation().y();
  pose->pose.orientation.z = tf_tf2.getRotation().z();
  pose->pose.orientation.w = tf_tf2.getRotation().w();
  return true;
}



void PlannerRos::planFromCurrentRobotPose() {
  std::string map_frame;
  {
    std::lock_guard<std::mutex> lock(map_queue_mutex_);
    map_frame = map_queue_.header.frame_id;
  }
  if (map_frame.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No map frame received yet. Not planning.");
    publishFeedback(Feedback::NO_MAP);
    return;
  }

  // Convert goal pose to map frame (map might drift w.r.t goal).
  geometry_msgs::msg::PoseStamped pose_goal_transformed;
  try {
    std::lock_guard<std::mutex> lock(pose_goal_mutex_);
    if (pose_goal_.header.frame_id == map_frame) {
      pose_goal_transformed = pose_goal_;
    } else {
      pose_goal_.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      tf_buffer_.transform(pose_goal_,
                           pose_goal_transformed,
                           map_frame,
                           tf2::durationFromSec(0.1));
    }
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(node_->get_logger(), "Could not transform goal pose to map frame. Not planning.");
    publishFeedback(Feedback::NO_GOAL_TF);
    return;
  }

  ob::ScopedState<> goal = converter_.poseRosToOmpl(pose_goal_transformed);

  publishFeedback(Feedback::PLANNING);
  const auto result = updateMapAndPlanFromCurrentRobotPose(goal);
  visualizePlannerGraph("", false);
  visualizePlannerGraph("invalid/", true);

  switch (result) {
    case PlannerStatus::INVALID_GOAL: publishFeedback(Feedback::INVALID_GOAL); break;
    case PlannerStatus::INVALID_START: publishFeedback(Feedback::INVALID_START); break;
    case PlannerStatus::NO_MAP: publishFeedback(Feedback::NO_MAP); break;
    case PlannerStatus::NOT_SOLVED: publishFeedback(Feedback::NO_SOLUTION); break;
    case PlannerStatus::SOLVED: publishFeedback(Feedback::FOUND_SOLUTION); break;
    case PlannerStatus::UNKNOWN: RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown planner feedback. Something is wrong!");
  }

  // Publish path if successful and we did not reach the goal.
  if (result == PlannerStatus::SOLVED && planning_continuously_) {
    auto plan_ros = converter_.pathOmplToRos(getSolutionPath(params_->planner.simplify_solution));
    plan_ros.header.frame_id = planning_map_header_.frame_id;
    plan_ros.header.stamp = node_->now();
    publishPath(plan_ros);
  }
}



void PlannerRos::publishFeedback(FeedbackStatus feedback) const {
  if (!rclcpp::ok() || !active_goal_ || !active_goal_->is_active()) {
    return;
  }
  auto feedback_msg = std::make_shared<Feedback>();
  feedback_msg->status = feedback;
  try {
    active_goal_->publish_feedback(feedback_msg);
  } catch (const rclcpp::exceptions::RCLError& ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Failed to publish action feedback: %s", ex.what());
  }
}



rclcpp_action::GoalResponse PlannerRos::goalCallback(const rclcpp_action::GoalUUID&,
                                                     std::shared_ptr<const PlanToGoal::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}



rclcpp_action::CancelResponse PlannerRos::cancelGoalCallback(const std::shared_ptr<GoalHandlePlanToGoal> goal_handle) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Stop continuous planning requested.");
  stopPlanningContinuously();
  if (goal_handle) {
    auto logger = node_->get_logger();
    std::thread([goal_handle, logger]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (!goal_handle->is_canceling()) {
        return;
      }
      try {
        goal_handle->canceled(std::make_shared<PlanToGoal::Result>());
      } catch (const rclcpp::exceptions::RCLError& ex) {
        RCLCPP_DEBUG(logger, "Failed to mark ART goal canceled: %s", ex.what());
      }
    }).detach();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}



void PlannerRos::acceptedGoalCallback(const std::shared_ptr<GoalHandlePlanToGoal> goal_handle) {
  active_goal_ = goal_handle;
  goalPoseCallback(goal_handle->get_goal());
}



void PlannerRos::goalPoseCallback(std::shared_ptr<const PlanToGoal::Goal> goal_msg) {
  std::lock_guard<std::mutex> lock(pose_goal_mutex_);

  pose_goal_ = goal_msg->goal;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Received goal pose in frame " << pose_goal_.header.frame_id);
  publishFeedback(Feedback::GOAL_RECEIVED);
  if (!planning_continuously_) {
    std::lock_guard<std::mutex> lock(planning_thread_mutex_);
    planning_continuously_ = true;
    continuous_planning_thread_ = std::thread(&PlannerRos::planContinuouslyThread, this);
  }

}



bool PlannerRos::transformRosPoseToMapFrame(const geometry_msgs::msg::PoseStamped& in,
                                            geometry_msgs::msg::PoseStamped& out) const {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(map_queue_.header.frame_id,
                                    tf2::getFrameId(in),
                                    tf2::getTimestamp(in),
                                    tf2::durationFromSec(0.01));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    return false;
  }
  tf2::doTransform(in, out, tf);
  return true;
}



void PlannerRos::getPlanService(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
                                std::shared_ptr<nav_msgs::srv::GetPlan::Response> res) {
  if (!transformRosPoseToMapFrame(req->start, req->start)) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform start pose to map frame. Assuming it's already map.");
  }
  if (!transformRosPoseToMapFrame(req->goal, req->goal)) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform goal pose to map frame. Assuming it's already map.");
  }

  ob::ScopedState<> start = converter_.poseRosToOmpl(req->start);
  ob::ScopedState<> goal = converter_.poseRosToOmpl(req->goal);

  const auto result = updateMapAndPlan(start, goal);

  if (result == PlannerStatus::SOLVED) {
    res->plan = converter_.pathOmplToRos(getSolutionPath(params_->planner.simplify_solution));
    res->plan.header.frame_id = planning_map_header_.frame_id;
    res->plan.header.stamp = node_->now();
    publishPath(res->plan);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Could not find a plan for GetPlan request.");
  }
}



void PlannerRos::publishPath(nav_msgs::msg::Path path) {
  // Publish regular ROS message.
  path_pub_->publish(path);
  visualizer_.visualizePathCollisions(path);
}

PlannerRos::~PlannerRos() {
  // Wait for planning thread to finish.
  if (planning_continuously_) {
    std::cout << "Stopping continuous planning before shutdown." << std::endl;
    stopPlanningContinuously();
    publishFeedback(Feedback::NODE_SHUTDOWN);
    if (active_goal_ && active_goal_->is_active()) {
      active_goal_->abort(std::make_shared<PlanToGoal::Result>());
    }
  }
}



void PlannerRos::visualizePlannerGraph(const std::string& ns_prefix, bool get_invalid) {
  ob::PlannerData dat(ss_->getSpaceInformation());
  if (params_->planner.name == "prm_motion_cost") {
    ss_->getPlanner()->as<PRMMotionCost>()->getPlannerData(dat, get_invalid);
  } else {
    if (get_invalid) {
      return;
    }
    ss_->getPlannerData(dat);
  }

  visualizer_.visualizePlannerGraph(dat, planning_map_header_.frame_id, ns_prefix);
}



PlannerRos::PlannerRos(const rclcpp::Node::SharedPtr& node)
    : Planner(loadRosParameters(node)),
      node_(node),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_, node_),
      visualizer_(node_, params_),
      converter_(space_) {

  map_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
      "~/elevation_map", 1, std::bind(&PlannerRos::mapCallback, this, std::placeholders::_1));
  plan_act_srv_ = rclcpp_action::create_server<PlanToGoal>(
      node_,
      "~/plan_to_goal",
      std::bind(&PlannerRos::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlannerRos::cancelGoalCallback, this, std::placeholders::_1),
      std::bind(&PlannerRos::acceptedGoalCallback, this, std::placeholders::_1));
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
  map_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("~/map", rclcpp::QoS(1).transient_local());
  timer_pub_ = node_->create_publisher<std_msgs::msg::Float64>("~/planning_time", 1);
  plan_srv_ = node_->create_service<nav_msgs::srv::GetPlan>(
      "~/plan", std::bind(&PlannerRos::getPlanService, this, std::placeholders::_1, std::placeholders::_2));

  // Cost service handling.
  if (params_->planner.name == "prm_motion_cost") {
    // Wait for services.
    cost_srv_client_ = node_->create_client<art_planner_motion_cost::srv::CostQuery>("~/cost_query");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for cost service...");
    while (rclcpp::ok() && !cost_srv_client_->wait_for_service(1s)) {}
    RCLCPP_INFO_STREAM(node_->get_logger(), "Service is up.");
    cost_no_update_srv_client_ = node_->create_client<art_planner_motion_cost::srv::CostQuery>("~/cost_query_no_update");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for cost no update service...");
    while (rclcpp::ok() && !cost_no_update_srv_client_->wait_for_service(1s)) {}
    RCLCPP_INFO_STREAM(node_->get_logger(), "Service is up.");

    // Pass service call to planner.
    auto cost_func = [this](const MotionCostObjective::EdgeMatrix& edge_matrix,
                            MotionCostObjective::EdgeMatrix* edge_costs,
                            rclcpp::Client<art_planner_motion_cost::srv::CostQuery>::SharedPtr client) {
      auto request = std::make_shared<art_planner_motion_cost::srv::CostQuery::Request>();
      request->header = planning_map_header_;
      // TODO: Find out if we can do this without data copying.
      request->query_poses.assign(edge_matrix.data(),
                                     edge_matrix.data()+edge_matrix.cols()*edge_matrix.rows());

      // Query service.
      auto future = client->async_send_request(request);
      const auto success = future.wait_for(10s) == std::future_status::ready;

      // No need to copy data if service failed.
      if (!success) {
        return false;
      }
      const auto response = future.get();

      // Write result to cost matrix.
      for (size_t i = 0; i < edge_costs->rows(); ++i) {
        (*edge_costs)(i, 0) = response->cost_power[i];
        (*edge_costs)(i, 1) = response->cost_time[i];
        (*edge_costs)(i, 2) = response->cost_risk[i];
      }

      return success;
    };
    ss_->getPlanner()->as<PRMMotionCost>()->setMaintainer(
          std::unique_ptr<PRMMotionCostMaintainer>(
              new PRMMotionCostMaintainer(map_, params_, std::make_unique<MotionCostObjective::MotionCostFunc>(
                                                              std::bind(cost_func, std::placeholders::_1, std::placeholders::_2, cost_srv_client_)))));
    ss_->setOptimizationObjective(
              std::make_shared<MotionCostObjective>(ss_->getSpaceInformation(),
                                                    params_,
                                                    std::make_unique<MotionCostObjective::MotionCostFunc>(
                                                         std::bind(cost_func, std::placeholders::_1, std::placeholders::_2, cost_no_update_srv_client_))));
  }
}



void PlannerRos::updateMap() {
  std::lock_guard<std::mutex> lock(map_queue_mutex_);

  if (!map_queue_.map) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "No new map received since last planning call.");
  } else {
    planning_map_info_ = map_queue_.info;
    planning_map_header_ = map_queue_.header;
    setMap(std::move(map_queue_.map));
  }
}



void PlannerRos::publishMap() const {
  if (map_pub_->get_subscription_count() > 0) {
    auto out_msg = grid_map::GridMapRosConverter::toMessage(map_->getMap());
    out_msg->header = planning_map_header_;
    out_msg->info = planning_map_info_;
    map_pub_->publish(*out_msg);
  }
}



void PlannerRos::publishTiming(const double& timing) const {
  std_msgs::msg::Float64 msg;
  msg.data = timing;
  timer_pub_->publish(msg);
}



PlannerStatus PlannerRos::updateMapAndPlan(const ob::ScopedState<>& start,
                                           const ob::ScopedState<>& goal) {
  updateMap();

  ss_->clear();

  const auto result = plan(start, goal);

  publishMap();

  return result;
}



void PlannerRos::logStartStateDiagnosis(const geometry_msgs::msg::PoseStamped& pose_robot,
                                        const ob::ScopedState<>& start) const {
  const grid_map::Position position(pose_robot.pose.position.x, pose_robot.pose.position.y);
  double elevation = std::numeric_limits<double>::quiet_NaN();
  double elevation_masked = std::numeric_limits<double>::quiet_NaN();
  bool inside_map = false;
  bool has_elevation_masked = false;

  try {
    inside_map = map_->isInside(position);
    if (inside_map) {
      elevation = map_->getHeightAtPosition(position);
      const auto& grid_map = map_->getMap();
      has_elevation_masked = grid_map.exists("elevation_masked");
      if (has_elevation_masked) {
        elevation_masked = grid_map.atPosition("elevation_masked", position);
      }
    }
  } catch (const std::exception& ex) {
    RCLCPP_WARN(node_->get_logger(), "起点诊断读取地图失败: %s", ex.what());
  }

  const bool start_valid = ss_->getSpaceInformation()->isValid(start.get());
  const double feet_center_z = pose_robot.pose.position.z;
  const double estimated_base_z = feet_center_z - params_->robot.feet.offset.z;

  if (start_valid) {
    RCLCPP_INFO(node_->get_logger(),
                "起点诊断: 起点有效。frame=%s, feet中心=(%.3f, %.3f, %.3f), 估算base_z=%.3f, elevation=%.3f",
                pose_robot.header.frame_id.c_str(),
                pose_robot.pose.position.x,
                pose_robot.pose.position.y,
                feet_center_z,
                estimated_base_z,
                elevation);
    return;
  }

  RCLCPP_WARN(node_->get_logger(),
              "起点诊断: 起点无效。frame=%s, feet中心=(%.3f, %.3f, %.3f), 估算base_z=%.3f, "
              "地图内=%s, elevation=%.3f, elevation_masked=%s%.3f, "
              "feet.offset.z=%.3f, feet.reach.z=%.3f, torso.offset.z=%.3f, torso.height=%.3f",
              pose_robot.header.frame_id.c_str(),
              pose_robot.pose.position.x,
              pose_robot.pose.position.y,
              feet_center_z,
              estimated_base_z,
              inside_map ? "true" : "false",
              elevation,
              has_elevation_masked ? "" : "无/",
              elevation_masked,
              params_->robot.feet.offset.z,
              params_->robot.feet.reach.z,
              params_->robot.torso.offset.z,
              params_->robot.torso.height);

  if (!inside_map) {
    RCLCPP_WARN(node_->get_logger(), "起点无效原因提示: feet中心不在当前grid map范围内，请检查map坐标系、elevation_mapping_cupy地图中心和TF。");
  } else if (has_elevation_masked && !std::isfinite(elevation_masked)) {
    RCLCPP_WARN(node_->get_logger(), "起点无效原因提示: elevation_masked无有限高程，该位置可能被认为不可通行或未知。");
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "起点无效原因提示: 地形高程应落在feet支撑框范围内。当前feet支撑框z范围约[%.3f, %.3f]，elevation_masked约%.3f。",
                feet_center_z - params_->robot.feet.reach.z * 0.5,
                feet_center_z + params_->robot.feet.reach.z * 0.5,
                elevation_masked);
  }
}



PlannerStatus PlannerRos::updateMapAndPlanFromCurrentRobotPose(const ob::ScopedState<>& goal) {
  updateMap();

  ss_->clear();
  ss_->setup();
  if (params_->planner.name == "prm_motion_cost") {
    auto planner = ss_->getPlanner()->as<PRMMotionCost>();
    planner->sampleGraph();
  }

  // Get robot pose.
  geometry_msgs::msg::PoseStamped pose_robot;
  if (!getCurrentRobotPose(&pose_robot)) {
    return PlannerStatus::UNKNOWN;
  }

  ob::ScopedState<> start = converter_.poseRosToOmpl(pose_robot);
  logStartStateDiagnosis(pose_robot, start);

  const auto result = plan(start, goal);

  publishMap();

  return result;
}
