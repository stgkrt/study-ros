cmake_minimum_required(VERSION 3.5)
project(my_first_package)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(hello src/hello.cpp)
ament_target_dependencies(hello rclcpp)
install(TARGETS hello DESTINATION lib/${PROJECT_NAME})

add_executable(hello_loop src/hello_loop.cpp)
ament_target_dependencies(hello_loop rclcpp)
install(TARGETS hello_loop DESTINATION lib/${PROJECT_NAME})

set(NODES my_publisher;my_subscriber)
foreach(target IN LISTS NODES)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(my_turtle_controller src/my_turtle_controller.cpp)
ament_target_dependencies(my_turtle_controller rclcpp geometry_msgs)
install(TARGETS my_turtle_controller DESTINATION lib/${PROJECT_NAME})

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
