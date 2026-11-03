cmake_minimum_required(VERSION 3.8)
project(amr_manipulator_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Loads the boxes of the warehouse SDF into the planning scene (started by move_group.launch.py)
install(PROGRAMS scripts/scene_from_sdf.py DESTINATION lib/${PROJECT_NAME})

ament_package()
