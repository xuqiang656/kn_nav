cmake_minimum_required(VERSION 3.8)
project(pct_planner)

find_package(ament_cmake REQUIRED)

# Install the shell wrapper as the executable (ros2 run pct_planner run_ros2_global_planner)
# The wrapper sets LD_LIBRARY_PATH and PCT_ROOT, then exec's the actual Python script.
install(PROGRAMS
  scripts/run_ros2_global_planner_wrapper
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_ros2_global_planner
)

# Install the Python script alongside (wrapper calls it via PCT_ROOT/scripts/)
install(PROGRAMS
  scripts/run_ros2_global_planner
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install parameter files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
