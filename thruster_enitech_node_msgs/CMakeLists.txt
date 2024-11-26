cmake_minimum_required(VERSION 3.8)
project(thruster_enitech_node_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/BG149Temperature.msg"
  "msg/BG149TemperatureArray.msg"
  "msg/CmdRaw.msg"
  "msg/CmdSpeed.msg"
  "msg/Emergency.msg"
  "msg/Heartbeat.msg"
  "msg/Joints.msg"
  "msg/JointSample.msg"
  "msg/MonitoringVariables.msg"
  "msg/Status.msg"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} 
                                rosidl_typesupport_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
