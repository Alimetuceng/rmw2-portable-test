cmake_minimum_required(VERSION 3.8)
project(my_rmw_tests_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_service test/test_service.cpp)
target_link_libraries(test_service rclcpp::rclcpp)
ament_target_dependencies(test_service rclcpp example_interfaces)

ament_package()

