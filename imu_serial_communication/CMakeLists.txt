cmake_minimum_required(VERSION 3.5)
project(imu_serial_communication)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -march=native -DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

include_directories(include)

set(PORT_HANDLER_SOURCES src/PortHandler.cpp)
add_library(port_handler_library SHARED ${PORT_HANDLER_SOURCES})
install(TARGETS port_handler_library DESTINATION lib)

add_executable(port_test src/port_test.cpp)
target_link_libraries(port_test port_handler_library)
install(TARGETS port_test DESTINATION lib/${PROJECT_NAME})

set(imu_publisher_dependencies "rclcpp" "sensor_msgs")
add_executable(imu_publisher src/IMUPublisher.cpp)
ament_target_dependencies(imu_publisher ${imu_publisher_dependencies})
target_link_libraries(imu_publisher port_handler_library)
install(TARGETS imu_publisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
