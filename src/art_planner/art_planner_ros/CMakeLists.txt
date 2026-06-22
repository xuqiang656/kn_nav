cmake_minimum_required(VERSION 3.8)
project(art_planner_ros)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(art_planner REQUIRED)
find_package(art_planner_motion_cost REQUIRED)
find_package(art_planner_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(ODE REQUIRED ode)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  art_planner
  art_planner_motion_cost
  art_planner_msgs
  geometry_msgs
  grid_map_msgs
  grid_map_ros
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_library(${PROJECT_NAME}
  src/planner_ros.cpp
  src/utils.cpp
  src/converter.cpp
  src/visualizer.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_node src/planner_ros_node.cpp)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})
ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_DEPENDS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plan_to_goal_client.py
  scripts/path_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
