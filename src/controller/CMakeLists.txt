cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)

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

# 声明插件
pluginlib_export_plugin_description_file(controller_interface plugins.xml)
# 声明插件
pluginlib_export_plugin_description_file(hardware_interface Mycontrol_plugins.xml)

add_library(visual_servoing_controller SHARED
  src/visual_servoing_controller.cpp
)

add_library(my_controller SHARED 
  src/Mycontroller.cpp
)



target_include_directories(visual_servoing_controller PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(visual_servoing_controller 
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  hardware_interface 
  controller_interface
  pluginlib
  controller_manager_msgs
  joint_trajectory_controller
  robot_interfaces
  std_msgs
  urdf 
  control_msgs
  builtin_interfaces
  kdl_parser
  orocos_kdl
  moveit_msgs
  Eigen3
)

ament_target_dependencies(my_controller 
  rclcpp
  hardware_interface
  pluginlib
  controller_manager_msgs
  urdf
  robot_interfaces
)

install(TARGETS visual_servoing_controller 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS my_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装插件描述文件
install(FILES plugins.xml Mycontrol_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
