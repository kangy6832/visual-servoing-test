# cmake_minimum_required(VERSION 3.8)
# project(control_pack)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(orocos_kdl REQUIRED)

# include_directories(${Eigen3_INCLUDE_DIRS})
# include_directories(${orocos_kdl_INCLUDE_DIR})
# include_directories(../include/control_pack)

# add_executable(${PROJECT_NAME} src/robot_kinematics_kdl.cpp)

# ament_target_dependencies(${PROJECT_NAME} 
#   Eigen3
  
# )




# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.10)
project(RobotKinematicsKDL VERSION 1.0.0 LANGUAGES CXX)

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "")
    add_compile_options(-O3 -DNDEBUG)
endif()

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror=return-type
    -Werror=uninitialized
)

# 查找依赖包
find_package(orocos_KDL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# 打印配置信息
message(STATUS "KDL found: ${KDL_VERSION}")
message(STATUS "Eigen3 found: ${Eigen3_VERSION}")

# 创建库目标
add_library(${PROJECT_NAME}
    STATIC
    src/robot_kinematics_kdl.cpp
    include/robot_kinematics_kdl.hpp
)

# 设置包含目录
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 链接依赖
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        orocos-kdl
        Eigen3::Eigen
)

# 设置输出目录
set_target_properties(${PROJECT_NAME}
    PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

# 安装配置
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT library
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

# 测试配置
enable_testing()
add_subdirectory(test)

# Doxygen文档配置（可选）
find_package(Doxygen)
if(Doxygen_FOUND)
    add_subdirectory(doc)
endif()

# 打印构建摘要
message(STATUS "")
message(STATUS "======================================")
message(STATUS "RobotKinematicsKDL Build Summary")
message(STATUS "======================================")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "KDL: ${KDL_VERSION}")
message(STATUS "Eigen3: ${Eigen3_VERSION}")
message(STATUS "======================================")
