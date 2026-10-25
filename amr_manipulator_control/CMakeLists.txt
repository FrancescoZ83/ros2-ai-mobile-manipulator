cmake_minimum_required(VERSION 3.8)
project(amr_manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

add_library(amr_manipulator_control SHARED src/arm_model_controller.cpp)
target_compile_features(amr_manipulator_control PUBLIC cxx_std_17)
target_include_directories(amr_manipulator_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amr_manipulator_control
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  kdl_parser orocos_kdl)
# Belt and braces: make sure the KDL library itself is linked
target_link_libraries(amr_manipulator_control ${orocos_kdl_LIBRARIES})

# Tell pluginlib that this package provides controller_interface plugins
pluginlib_export_plugin_description_file(controller_interface amr_manipulator_control_plugins.xml)

install(TARGETS amr_manipulator_control
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(amr_manipulator_control)
ament_export_dependencies(controller_interface hardware_interface pluginlib
  rclcpp rclcpp_lifecycle kdl_parser orocos_kdl)
ament_package()
