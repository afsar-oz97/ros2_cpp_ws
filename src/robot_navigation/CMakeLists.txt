cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces (
  ${PROJECT_NAME}
  "action/Navigation.action"
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  action_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(navigation_action_server src/navigation_action_server.cpp)
ament_target_dependencies(navigation_action_server rclcpp std_msgs sensor_msgs geometry_msgs action_msgs rclcpp_action)
target_link_libraries(navigation_action_server ${cpp_typesupport_target})

add_executable(navigation_action_client src/navigation_action_client.cpp)
ament_target_dependencies(navigation_action_client rclcpp std_msgs sensor_msgs geometry_msgs action_msgs rclcpp_action)
target_link_libraries(navigation_action_client ${cpp_typesupport_target})

install (TARGETS 
  navigation_action_server
  navigation_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

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
