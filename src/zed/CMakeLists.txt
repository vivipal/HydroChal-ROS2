cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME zed)
project(${PROJECT_NAME} CXX)

############################################################################
# Flags

cmake_policy(SET CMP0054 NEW)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
set(${PROJECT_NAME}_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "${${PROJECT_NAME}_CXX_FLAGS} -Wall -pedantic -g")

if(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s -flto") # File size optimization
    message("* Release mode")
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message("* Debug mode")
    SET(CMAKE_DEBUG_POSTFIX  "d")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s -flto") # File size optimization
    message("* Release mode")
endif()

# Allows to start application by double clicking the icon on Ubuntu
set ( CMAKE_CXX_FLAGS "-no-pie" )

# CPU information
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Raspberry Pi 4 - OS 64bit
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("* Optimizing for ${CMAKE_SYSTEM_PROCESSOR}")
  add_compile_options(-mtune=cortex-a72 -mcpu=cortex-a72)
  add_definitions(-DEMBEDDED_ARM)
endif()

# Raspberry Pi 4 - OS 32bit
if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  message("* Optimizing for ${CMAKE_SYSTEM_PROCESSOR}")
  # add_compile_options(-mcpu=cortex-a72 -mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -mtune=cortex-a72)
  add_compile_options(-mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -mtune=cortex-a72)
  add_definitions(-DEMBEDDED_ARM)
endif()

# ROS stuff vodka
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################################################################
# Options
option(DEBUG_CAM_REG    "Add functions to log the values of the registers of camera"  OFF)

############################################################################
# Sources
set(SRC_VIDEO
    ${PROJECT_SOURCE_DIR}/src/videocapture.cpp
)

############################################################################
# Includes
set(HEADERS_VIDEO
    # Base
    ${PROJECT_SOURCE_DIR}/include/videocapture.hpp
    
    # Defines
    ${PROJECT_SOURCE_DIR}/include/defines.hpp
    ${PROJECT_SOURCE_DIR}/include/videocapture_def.hpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

############################################################################
# Required external libraries

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

############################################################################
# Generate libraries
if(DEBUG_CAM_REG)
    message("* Registers logging available")
    add_definitions(-DSENSOR_LOG_AVAILABLE)
endif()

message("* Video module available")
add_definitions(-DVIDEO_MOD_AVAILABLE)

find_package(LIBUSB 1 REQUIRED)
message(STATUS "libusb: include dir at ${LibUSB_INCLUDE_DIRS}")
message(STATUS "libusb: libraries at ${LibUSB_LIBRARIES}")
include_directories(${LibUSB_INCLUDE_DIRS})
set(SRC_FULL ${SRC_FULL} ${SRC_VIDEO})
set(HDR_FULL ${HDR_FULL} ${HEADERS_VIDEO})
set(DEP_LIBS ${DEP_LIBS} ${LibUSB_LIBRARIES})

add_library(${PROJECT_NAME} SHARED ${SRC_FULL} )
target_link_libraries( ${PROJECT_NAME}  ${DEP_LIBS})


############################################################################
# Install rules
set_target_properties(${PROJECT_NAME} PROPERTIES
    PUBLIC_HEADER "${HDR_FULL}"
)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/zed-open-capture)


############################################################################
# Generate nodes

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(interfaces REQUIRED)

# Set dependencies
set(dependencies
  rclcpp
  sensor_msgs
  stereo_msgs
  cv_bridge
  image_transport
  interfaces
)

##### OpenCV is required
find_package(OpenMP)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(DEPTH_STEREO ${PROJECT_NAME}_depth_stereo)
include_directories( ${PROJECT_SOURCE_DIR}/include)
add_executable(${DEPTH_STEREO} "${PROJECT_SOURCE_DIR}/src/zed_oc_depth_stereo.cpp")
set_target_properties(${DEPTH_STEREO} PROPERTIES PREFIX "")

if(OpenMP_CXX_FOUND)
    include_directories(${OpenMP_CXX_INCLUDE_DIRS})
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    target_link_libraries(${DEPTH_STEREO}
        OpenMP::OpenMP_CXX
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
    )
else()
    target_link_libraries(${DEPTH_STEREO}
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
    )
endif()

ament_target_dependencies(${DEPTH_STEREO} ${dependencies})

# Install C++ executables
install(TARGETS
  ${DEPTH_STEREO}
  DESTINATION lib/${PROJECT_NAME}
)

# Build Python nodes
ament_python_install_package(scripts)
install(PROGRAMS
  scripts/obstacle_detector.py
  scripts/tools.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
