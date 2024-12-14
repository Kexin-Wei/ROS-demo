cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

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

add_executable(my_cpp_node src/my_first_cpp_node.cpp)
ament_target_dependencies(my_cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(a_listener src/a_listener.cpp)
ament_target_dependencies(a_listener rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp std_msgs std_srvs)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hardware_status_subscriber src/hardware_status_subscriber.cpp)
ament_target_dependencies(hardware_status_subscriber rclcpp my_robot_interfaces)

add_executable(led_panel_node src/led_panel_node.cpp)
ament_target_dependencies(led_panel_node rclcpp my_robot_interfaces)

install(TARGETS 
  my_cpp_node
  robot_news_station
  a_listener
  number_counter
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hardware_status_subscriber
  led_panel_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
