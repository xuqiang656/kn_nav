#pragma once

#include <thread>
#include <unordered_map>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ompl/base/PlannerData.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <art_planner/params.h>



namespace art_planner {



class Visualizer {

  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_collision_pub_;

  std::thread collision_pub_thread_;

  ParamsConstPtr params_;

  int last_num_vertices{0};
  int last_num_new_vertices{0};
  int last_num_edges{0};
  int last_num_start_goal{0};
  int last_num_path_collisions{100};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr getGraphPublisher(const std::string& ns_prefix);

  void addVertices(const ompl::base::PlannerData& dat,
                   visualization_msgs::msg::MarkerArray& array);

  void addEdges(const ompl::base::PlannerData& dat,
                visualization_msgs::msg::MarkerArray& array);

  void addStartGoal(const ompl::base::PlannerData& dat,
                    visualization_msgs::msg::MarkerArray& array);

  void visualizeCollisionBoxesThread(const visualization_msgs::msg::MarkerArray &array);

  void visualizeCollisionBoxes();

public:

  Visualizer(const rclcpp::Node::SharedPtr& node, const ParamsConstPtr& params);

  ~Visualizer();

  void visualizePlannerGraph(const ompl::base::PlannerData& dat,
                             const std::string &frame_id,
                             const std::string& ns_prefix = "");


  void visualizePathCollisions(const nav_msgs::msg::Path& path);

};



}
