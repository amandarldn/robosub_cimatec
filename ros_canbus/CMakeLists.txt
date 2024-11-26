cmake_minimum_required(VERSION 3.8)
project(ros_canbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros_driver_base REQUIRED)

set(CANBUS_SOURCE_FILES
    src/Driver.cpp 
    src/Driver2Web.cpp 
    src/DriverHico.cpp 
    src/DriverHicoPCI.cpp 
    src/DriverNetGateway.cpp
    src/DriverSocket.cpp 
    src/DriverEasySYNC.cpp
)

set(CANBUS_HEADERS_FILES
    include/ros_canbus/Driver.hpp 
    include/ros_canbus/time.hpp
    include/ros_canbus/Message.hpp
    include/ros_canbus/Driver2Web.hpp 
    include/ros_canbus/DriverHico.hpp 
    include/ros_canbus/DriverHicoPCI.hpp 
    include/ros_canbus/DriverNetGateway.hpp
    include/ros_canbus/DriverSocket.hpp 
    include/ros_canbus/DriverEasySYNC.hpp
)

add_library(ros_canbus
    ${CANBUS_SOURCE_FILES}
    ${CANBUS_HEADERS_FILES}
)

target_compile_features(ros_canbus PUBLIC c_std_17 cxx_std_20)
target_include_directories(ros_canbus PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

target_compile_definitions(ros_canbus
   PRIVATE "FLATFISH_ROS_THRUSTERS_ENITECH_BUILDING_LIBRARY")

ament_target_dependencies(
    ros_canbus
    ros_driver_base
)

install(DIRECTORY 
  include/ 
  DESTINATION include)

install(
    TARGETS ros_canbus 
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

add_executable(canbus-easysync src/tools/MainEasySYNC.cpp)
add_executable(canbus-monitor src/tools/MainMonitor.cpp)
add_executable(hico_tool src/tools/hcantool.c)
add_executable(canbus-reset src/tools/MainReset.cpp)
add_executable(canbus-send src/tools/MainSend.cpp)

target_include_directories(canbus-easysync PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(canbus-monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(hico_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(canbus-reset PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(canbus-send PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(canbus-easysync ros_canbus)
target_link_libraries(canbus-monitor ros_canbus)
target_link_libraries(hico_tool ros_canbus)
target_link_libraries(canbus-reset ros_canbus)
target_link_libraries(canbus-send ros_canbus)

install(TARGETS 
  canbus-easysync
  canbus-monitor
  hico_tool
  canbus-reset
  canbus-send
  DESTINATION lib/${PROJECT_NAME}
)

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
ament_export_dependencies(ros_driver_base)

ament_package()
