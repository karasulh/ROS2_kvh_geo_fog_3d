# Copyright (c) 2019, The MITRE Corporation
# All rights reserved.
#
# Sections of this project contains content developed by The MITRE Corporation.
# If this code is used in a deployment or embedded within another project,
# it is requested that you send an email to opensource@mitre.org in order to
# let us know where this software is being used.

cmake_minimum_required(VERSION 3.8)
project(kvh_geo_fog_3d_rviz)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(kvh_geo_fog_3d_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

# QT
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
qt5_wrap_cpp(QT_MOC include/kvh_geo_fog_3d_status_panel.hpp include/kvh_status_painter.hpp)

add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/kvh_geo_fog_3d_status_panel.cpp
  src/kvh_status_painter.cpp
  ${QT_MOC}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs kvh_geo_fog_3d_msgs rviz2 rviz_common ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(
  ${PROJECT_NAME} 
  ${QT_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
