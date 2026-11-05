cmake_minimum_required(VERSION 3.22)
project(e6_jazzy_cpp)
find_package(ament_cmake REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(parameter_traits REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
set(dependencies control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
    moveit_ros_planning_interface pluginlib rclcpp rclcpp_components rclcpp_lifecycle
    realtime_tools sensor_msgs std_msgs std_srvs tf2_eigen trajectory_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
add_executable(servo_cpp_adapter servo_cpp_adapter.cpp orientation_assist.cpp)
if(E6_DIRECTIONAL_SERVO_SOURCE)
  add_library(e6_directional_servo STATIC
      ${E6_DIRECTIONAL_SERVO_SOURCE}/src/collision_monitor.cpp
      ${E6_DIRECTIONAL_SERVO_SOURCE}/src/servo.cpp
      ${E6_DIRECTIONAL_SERVO_SOURCE}/src/utils/common.cpp
      ${E6_DIRECTIONAL_SERVO_SOURCE}/src/utils/command.cpp)
  target_include_directories(e6_directional_servo PUBLIC ${E6_DIRECTIONAL_SERVO_SOURCE}/include)
  target_link_libraries(e6_directional_servo moveit_servo::moveit_servo_lib_parameters parameter_traits::parameter_traits)
  ament_target_dependencies(e6_directional_servo ${dependencies})
  target_compile_features(e6_directional_servo PUBLIC cxx_std_17)
  target_link_libraries(servo_cpp_adapter e6_directional_servo)
else()
  target_link_libraries(servo_cpp_adapter moveit_servo::moveit_servo_lib_cpp)
endif()
target_link_libraries(servo_cpp_adapter
    moveit_servo::moveit_servo_lib_parameters parameter_traits::parameter_traits)
ament_target_dependencies(servo_cpp_adapter ${dependencies})
target_compile_features(servo_cpp_adapter PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # Bound compiler heap growth on the 3 GiB X5. These options control GCC's
  # internal garbage collector only; generated code/optimization stays -O3.
  target_compile_options(servo_cpp_adapter PRIVATE
    --param=ggc-min-expand=10 --param=ggc-min-heapsize=32768)
endif()
