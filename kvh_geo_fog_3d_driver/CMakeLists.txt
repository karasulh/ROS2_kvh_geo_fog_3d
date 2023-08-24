cmake_minimum_required(VERSION 3.8)
project(kvh_geo_fog_3d_driver)

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

#If diagnostic_updater dependencies gives error, then you should install it from terminal:
#sudo apt-get install ros-humble-diagnostic-updater

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kvh_geo_fog_3d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)


## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#  cfg/${PROJECT_NAME}.cfg
#)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include include/geo_fog_sdk/ include/rs232/ include/ 
)

add_library(kvh_driver_lib 
  geo_fog_sdk/an_packet_protocol.c
  geo_fog_sdk/rs232/rs232.c
  geo_fog_sdk/spatial_packets.c
  src/kvh_diagnostics_container.cpp
  src/kvh_driver/driver_main.cpp
  src/kvh_driver/decode_packets.cpp
  src/kvh_driver/device_configuration.cpp
  src/kvh_driver/kvh_global_vars.cpp
  src/kvh_driver/packet_storage.cpp
)

target_include_directories(kvh_driver_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kvh_driver_lib 
  diagnostic_updater ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


add_executable(kvh_geo_fog_3d_driver_node 
  src/kvh_geo_fog_3d_node.cpp 
  src/packet_publisher.cpp 
) 


target_include_directories(kvh_geo_fog_3d_driver_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kvh_geo_fog_3d_driver_node 
  rclcpp geometry_msgs diagnostic_msgs diagnostic_updater
                          kvh_geo_fog_3d_msgs nav_msgs sensor_msgs std_msgs tf2_ros 
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} #dynamic_reconfigure
) 

add_executable(determine_baud_node 
src/utility/determine_baud_node.cpp
)

target_include_directories(determine_baud_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(determine_baud_node 
  rclcpp ${THIS_PACKAGE_INCLUDE_DEPENDS} 
) 

#Need to link library and node to work with local library and node
target_link_libraries(
  kvh_geo_fog_3d_driver_node 
  kvh_driver_lib
) 

target_link_libraries(
  determine_baud_node 
  kvh_driver_lib
) 

install(
  TARGETS kvh_driver_lib
  DESTINATION lib
)

install(
  TARGETS kvh_geo_fog_3d_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY cfg launch
	DESTINATION share/${PROJECT_NAME}
) #for parameters and launch


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  
  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  
  # Ignore ROS's suggestions for:
  # * Comment spacing
  # * Parentesis whitespace
  set(AMENT_LINT_AUTO_IGNORE_FLAGS "--filter=-whitespace/comments,-whitespace/parens")


  ## Add gtest based cpp test target and link libraries
  include_directories(
  include test/include/
  )
  
  
  ament_lint_auto_find_test_dependencies()
  #ament_lint_auto_find_test_dependencies(EXCLUDE ${AMENT_LINT_AUTO_IGNORE_FLAGS})

     
  ament_add_gtest(${PROJECT_NAME}-some-test     
  test/init_vars.cpp
  test/device_configuration_test.cpp
  test/packet_storage_test.cpp
  test/kvh_driver_test.cpp
  )

  target_include_directories(
    ${PROJECT_NAME}-some-test  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  #target_include_directories(${PROJECT_NAME}-some-test PUBLIC test/include/)  # Need to specify directories in order to #include from them in the test

  ament_target_dependencies(${PROJECT_NAME}-some-test
       "rclcpp"
       ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
  target_link_libraries(${PROJECT_NAME}-some-test kvh_driver_lib)


endif()

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
