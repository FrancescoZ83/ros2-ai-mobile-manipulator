cmake_minimum_required(VERSION 3.8)
project(amr_manipulator_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds models rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
