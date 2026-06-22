#pragma once

#include <memory>
#include <thread>

#include <art_planner/planner.h>
#include <art_planner_motion_cost/srv/cost_query.hpp>
#include <art_planner_msgs/action/plan_to_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_msgs/msg/grid_map_info.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <art_planner_ros/converter.h>
#include <art_planner_ros/visualizer.h>
#include <art_planner/params.h>



namespace art_planner {



class PlannerRos : protected Planner {

 protected:

  // ROS members.
  using PlanToGoal = art_planner_msgs::action::PlanToGoal;
  using PlanningActionServer = rclcpp_action::Server<PlanToGoal>;
  using GoalHandlePlanToGoal = rclcpp_action::ServerGoalHandle<PlanToGoal>;
  using Feedback = PlanToGoal::Feedback;
  using FeedbackStatus = decltype(Feedback::status);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_srv_;
  rclcpp::Client<art_planner_motion_cost::srv::CostQuery>::SharedPtr cost_srv_client_;
  rclcpp::Client<art_planner_motion_cost::srv::CostQuery>::SharedPtr cost_no_update_srv_client_;
  PlanningActionServer::SharedPtr plan_act_srv_;
  std::shared_ptr<GoalHandlePlanToGoal> active_goal_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timer_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Visualizer visualizer_;

  // Planning members.
  geometry_msgs::msg::PoseStamped pose_goal_;
  mutable std::mutex pose_goal_mutex_;
  std::atomic<bool> planning_continuously_{false};
  std::thread continuous_planning_thread_;
  std::mutex planning_thread_mutex_;

  std_msgs::msg::Header planning_map_header_;
  grid_map_msgs::msg::GridMapInfo planning_map_info_;

  struct {
    std::unique_ptr<grid_map::GridMap> map;
    std_msgs::msg::Header header;
    grid_map_msgs::msg::GridMapInfo info;
  } map_queue_;
  mutable std::mutex map_queue_mutex_;

  Converter converter_;

  // Member functions.

  void stopPlanningContinuously();

  virtual void planContinuouslyThread();

  virtual void planFromCurrentRobotPose();

  nav_msgs::msg::Path getAndPublishPathFromTo(const geometry_msgs::msg::PoseStamped& pose_start,
                                         const geometry_msgs::msg::PoseStamped& pose_goal);

  bool getCurrentRobotPose(geometry_msgs::msg::PoseStamped *pose) const;

  bool transformRosPoseToMapFrame(const geometry_msgs::msg::PoseStamped& in,
                                  geometry_msgs::msg::PoseStamped& out) const;

  void publishFeedback(FeedbackStatus feedback) const;

  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr map_msg);

  rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const PlanToGoal::Goal> goal);

  rclcpp_action::CancelResponse cancelGoalCallback(const std::shared_ptr<GoalHandlePlanToGoal> goal_handle);

  void acceptedGoalCallback(const std::shared_ptr<GoalHandlePlanToGoal> goal_handle);

  void goalPoseCallback(std::shared_ptr<const PlanToGoal::Goal> goal);

  void getPlanService(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
                      std::shared_ptr<nav_msgs::srv::GetPlan::Response> res);

  bool planPath(const ob::ScopedState<>& start,
                const ob::ScopedState<>& goal,
                nav_msgs::msg::Path &path_out);

  virtual void publishPath(nav_msgs::msg::Path path);

  void visualizePlannerGraph(const std::string& ns_prefix="", bool get_invalid=false);

  void updateMap();

  void publishMap() const;

  void publishTiming(const double& timing) const;

  PlannerStatus updateMapAndPlan(const ob::ScopedState<>& start,
                                 const ob::ScopedState<>& goal);

  PlannerStatus updateMapAndPlanFromCurrentRobotPose(const ob::ScopedState<>& goal);

  void logStartStateDiagnosis(const geometry_msgs::msg::PoseStamped& pose_robot,
                              const ob::ScopedState<>& start) const;

public:

  ~PlannerRos();

  explicit PlannerRos(const rclcpp::Node::SharedPtr& node);

};



}
