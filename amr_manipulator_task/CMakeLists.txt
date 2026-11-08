cmake_minimum_required(VERSION 3.8)
project(amr_manipulator_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# One action per skill of the skill server (scripts/skill_server.py)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateTo.action"
  "action/PickArticle.action"
  "action/CarryArticle.action"
  "action/PlaceInBin.action"
  "action/Recover.action")
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(mission_executor src/mission_executor.cpp)
target_compile_features(mission_executor PUBLIC cxx_std_17)
ament_target_dependencies(mission_executor rclcpp rclcpp_action ament_index_cpp behaviortree_cpp_v3)
target_link_libraries(mission_executor "${typesupport}")

install(TARGETS mission_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})

ament_package()
