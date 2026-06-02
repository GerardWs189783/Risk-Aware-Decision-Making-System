cmake_minimum_required(VERSION 3.8)
project(rover_autonomy_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rover_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(bt_test src/test.cpp)
ament_target_dependencies(bt_test rclcpp behaviortree_cpp)

add_executable(autonomy_node src/autonomy_node.cpp)
ament_target_dependencies(autonomy_node rclcpp behaviortree_cpp ament_index_cpp rover_interfaces tf2 tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs trajectory_msgs)

# Install node
install(TARGETS 
bt_test 
autonomy_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_package()
