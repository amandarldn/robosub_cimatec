cmake_minimum_required(VERSION 3.8)
project(thruster_enitech_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_canbus REQUIRED)
find_package(ros_driver_base REQUIRED)

set(THRUSTER_ENITECH_SOURCE_FILES
    src/NMT.cpp
    src/Protocol.cpp
    src/SDO.cpp
    src/JointState.cpp
)

set(THRUSTER_ENITECH_HEADERS_FILES
    include/thruster_enitech_driver/Emergency.hpp 
    include/thruster_enitech_driver/NMT.hpp
    include/thruster_enitech_driver/Protocol.hpp 
    include/thruster_enitech_driver/Request.hpp
    include/thruster_enitech_driver/SDO.hpp 
    include/thruster_enitech_driver/Status.hpp
    include/thruster_enitech_driver/Float.hpp 
    include/thruster_enitech_driver/JointState.hpp
)

add_library(thruster_enitech_driver
    ${THRUSTER_ENITECH_SOURCE_FILES}
    ${THRUSTER_ENITECH_HEADERS_FILES}
)

target_compile_features(thruster_enitech_driver PUBLIC c_std_17 cxx_std_20)

target_include_directories(thruster_enitech_driver PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

target_compile_definitions(thruster_enitech_driver
   PRIVATE "THRUSTERS_ENITECH_DRIVER_BUILDING_LIBRARY")

  ament_target_dependencies(
    thruster_enitech_driver
    ros_driver_base
    ros_canbus
)

install(DIRECTORY 
  include/ 
  DESTINATION include)
  
install(
    TARGETS thruster_enitech_driver 
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
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

ament_export_libraries(
  thruster_enitech_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
