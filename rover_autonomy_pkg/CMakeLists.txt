cmake_minimum_required(VERSION 3.8)
project(rover_autonomy_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(bt_test src/test.cpp)
ament_target_dependencies(bt_test rclcpp behaviortree_cpp)

# Install node
install(TARGETS bt_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
