cmake_minimum_required(VERSION 3.8)

project(robotic_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/control/pbvs_controller.cpp
  src/control/velocity_controller.cpp
  src/kinematics/kinematics_solver.cpp
  src/planning/moveit_planner.cpp
  src/state_machine/state_machine.cpp
  src/trajectory/quintic_trajectory.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(pbvs_moveit_node src/pbvs_moveit_node.cpp)
target_link_libraries(pbvs_moveit_node ${PROJECT_NAME}_core)
ament_target_dependencies(pbvs_moveit_node rclcpp geometry_msgs sensor_msgs trajectory_msgs moveit_msgs)

install(TARGETS ${PROJECT_NAME}_core pbvs_moveit_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

