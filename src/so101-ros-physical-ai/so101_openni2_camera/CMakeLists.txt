cmake_minimum_required(VERSION 3.8)
project(so101_openni2_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# We strongly prefer the Orbbec-provided OpenNI2 SDK when available because
# Ubuntu's packaged OpenNI2 (2.2.x) can be incompatible with Orbbec's driver
# (commonly built/tested with OpenNI2 2.3.x).
#
# If you want to use the system OpenNI2 instead, configure with:
#   colcon build --packages-select so101_openni2_camera --cmake-args -DOPENNI2_SDK_DIR=
set(_OPENNI2_SDK_DEFAULT "")
set(_OPENNI2_ORBBEC_SDK_HINTS
  "$ENV{OPENNI2_SDK_DIR}"
  "$ENV{ORBBEC_OPENNI2_SDK_DIR}"
  "$ENV{HOME}/Orbbec/OpenNI2/sdk"
  "$ENV{HOME}/workspace/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk"
)
foreach(_sdk_hint IN LISTS _OPENNI2_ORBBEC_SDK_HINTS)
  if(_sdk_hint AND EXISTS "${_sdk_hint}/Include/OpenNI.h" AND EXISTS "${_sdk_hint}/libs/libOpenNI2.so")
    set(_OPENNI2_SDK_DEFAULT "${_sdk_hint}")
    break()
  endif()
endforeach()

set(OPENNI2_SDK_DIR "${_OPENNI2_SDK_DEFAULT}" CACHE PATH "OpenNI2 SDK directory (contains Include/ and libs/). Leave empty to use system OpenNI2.")

if(OPENNI2_SDK_DIR)
  set(OPENNI2_INCLUDE_DIR "${OPENNI2_SDK_DIR}/Include")
  set(OPENNI2_LIBRARY "${OPENNI2_SDK_DIR}/libs/libOpenNI2.so")
  if(NOT EXISTS "${OPENNI2_INCLUDE_DIR}/OpenNI.h" OR NOT EXISTS "${OPENNI2_LIBRARY}")
    message(FATAL_ERROR "OPENNI2_SDK_DIR is set but OpenNI.h / libOpenNI2.so not found under it: ${OPENNI2_SDK_DIR}")
  endif()
  message(STATUS "Using OpenNI2 from SDK: ${OPENNI2_SDK_DIR}")
else()
  # Fallback: OpenNI2 from system packages (libopenni2-dev + libopenni2-0).
  find_path(OPENNI2_INCLUDE_DIR
    NAMES OpenNI.h
    PATHS /usr/include
    PATH_SUFFIXES openni2 OpenNI2
  )

  find_library(OPENNI2_LIBRARY
    NAMES OpenNI2
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu
  )

  if(NOT OPENNI2_INCLUDE_DIR OR NOT OPENNI2_LIBRARY)
    message(FATAL_ERROR "OpenNI2 not found. Install libopenni2-dev (headers) and libopenni2-0 (runtime), or set OPENNI2_SDK_DIR.")
  endif()
  message(STATUS "Using OpenNI2 from system: ${OPENNI2_LIBRARY}")
endif()

get_filename_component(OPENNI2_LIBRARY_DIR "${OPENNI2_LIBRARY}" DIRECTORY)

add_executable(openni2_camera_node src/openni2_camera_node.cpp)
target_include_directories(openni2_camera_node PRIVATE ${OPENNI2_INCLUDE_DIR})
target_link_libraries(openni2_camera_node ${OPENNI2_LIBRARY})
ament_target_dependencies(openni2_camera_node rclcpp sensor_msgs std_msgs)
target_compile_features(openni2_camera_node PRIVATE cxx_std_17)

# Make the default OpenNI2 redistributable root align with the linked library dir.
target_compile_definitions(openni2_camera_node PRIVATE
  SO101_OPENNI2_DEFAULT_REDIST="${OPENNI2_LIBRARY_DIR}"
)

# Ensure the runtime loader can find libOpenNI2 when using the SDK build.
set_target_properties(openni2_camera_node PROPERTIES
  BUILD_RPATH "${OPENNI2_LIBRARY_DIR}"
  INSTALL_RPATH "${OPENNI2_LIBRARY_DIR}"
)

install(TARGETS openni2_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
