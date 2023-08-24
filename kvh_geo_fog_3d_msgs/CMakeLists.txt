cmake_minimum_required(VERSION 3.8)
project(kvh_geo_fog_3d_msgs)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
	"msg/KvhGeoFog3DDetailSatellites.msg"
  	"msg/KvhGeoFog3DECEFPos.msg"
  	"msg/KvhGeoFog3DLocalMagneticField.msg"
  	"msg/KvhGeoFog3DNorthSeekingInitStatus.msg"
  	"msg/KvhGeoFog3DOdometerState.msg"
  	"msg/KvhGeoFog3DRawGNSS.msg"
	"msg/KvhGeoFog3DRawSensors.msg"
	"msg/KvhGeoFog3DSatellites.msg"
	"msg/KvhGeoFog3DSystemState.msg"
	"msg/KvhGeoFog3DUTMPosition.msg"
	DEPENDENCIES std_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
