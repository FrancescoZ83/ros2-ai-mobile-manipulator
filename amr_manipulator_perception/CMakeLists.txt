cmake_minimum_required(VERSION 3.8)
project(amr_manipulator_perception)

find_package(ament_cmake REQUIRED)

# Perception nodes (Python): stereo depth now, recognition and pose later
install(PROGRAMS scripts/stereo_depth_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
