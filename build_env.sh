#!/bin/bash

colcon build --symlink-install
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix --share amr_manipulator_description)/..:$IGN_GAZEBO_RESOURCE_PATH
