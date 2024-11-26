cmake_minimum_required(VERSION 3.8)
project(thruster_enitech_node)

find_package(ament_cmake REQUIRED)
find_package(thruster_enitech_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_canbus REQUIRED)
find_package(thruster_enitech_node_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


add_library(ThrusterMonitor src/ThrusterMonitor.cpp)
target_include_directories(ThrusterMonitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(ThrusterNode src/ThrusterNode.cpp)
target_include_directories(ThrusterNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# target_include_directories(ThrusterMonitor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_include_directories(ThrusterNode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(ThrusterMonitor "thruster_enitech_node_msgs" "thruster_enitech_driver" "rclcpp" "std_msgs" "ros_canbus" "yaml-cpp")
ament_target_dependencies(ThrusterNode "thruster_enitech_node_msgs" "thruster_enitech_driver" "rclcpp" "std_msgs" "ros_canbus" "yaml-cpp")


add_executable(ThrusterMonitorExample src/ThrusterMonitorExample.cpp)
target_include_directories(ThrusterMonitorExample PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(ThrusterNodeExample src/ThrusterNodeExample.cpp)
target_include_directories(ThrusterNodeExample PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target_include_directories(ThrusterMonitorExample PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_include_directories(ThrusterNodeExample PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(ThrusterMonitorExample "rclcpp" "thruster_enitech_node_msgs" "thruster_enitech_driver" "ros_canbus" "std_msgs" "yaml-cpp")
ament_target_dependencies(ThrusterNodeExample "rclcpp" "thruster_enitech_node_msgs" "thruster_enitech_driver" "ros_canbus" "std_msgs" "yaml-cpp")

target_link_libraries(ThrusterMonitorExample ThrusterMonitor "yaml-cpp")
target_link_libraries(ThrusterNodeExample ThrusterNode "yaml-cpp")

install(DIRECTORY "config" "launch" DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ThrusterMonitor ThrusterNode
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ThrusterMonitorExample
    ThrusterNodeExample
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/ 
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(MAX_LINE_LENGTH "120")
  ament_lint_auto_find_test_dependencies()
endif()

if(BIR_ENABLE_DOCS)
  bir_add_doxygen_doc(DOC_TITLE "flatfish_ros_thrusters_enitech")
endif()

ament_export_include_directories(include)

ament_export_libraries(
  ThrusterMonitor ThrusterNode
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
