cmake_minimum_required(VERSION 3.8)
project(hydrochal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# sbus library for rpi
add_subdirectory(include/raspberry-sbus)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(ws src/WeatherStation.cpp)
add_executable(imu src/IMU.cpp)
add_executable(rx src/rx.cpp)

ament_target_dependencies(ws rclcpp std_msgs interfaces)
ament_target_dependencies(imu rclcpp std_msgs interfaces)
target_link_libraries(rx libsbus)
ament_target_dependencies(rx rclcpp interfaces)

install(TARGETS
  ws
  imu
  rx
  DESTINATION lib/${PROJECT_NAME})




ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/Rudder.py
  src/regulation.py
  src/untitled.py
  src/test.py
  
  DESTINATION lib/${PROJECT_NAME}
)

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
