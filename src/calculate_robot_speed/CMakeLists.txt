cmake_minimum_required(VERSION 3.8)
project(calculate_robot_speed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

#Run this first before executables if you are using this service on your executables
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OddEvenCheck.srv"
  "srv/ImageAngleChange.srv"
  DEPENDENCIES 
  sensor_msgs 
  ADD_LINTER_TESTS
  #DEPENDENCIES std_msgs   #dependencies should be listed here if the service uses types from other packages
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(rpm_publisher src/rpm_publisher.cpp)
ament_target_dependencies(rpm_publisher rclcpp std_msgs)

add_executable(speed_publisher src/speed_publisher.cpp)
ament_target_dependencies(speed_publisher rclcpp std_msgs)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp std_msgs)
target_link_libraries(service_server ${cpp_typesupport_target})

add_executable(client_server src/client_server.cpp)
ament_target_dependencies(client_server rclcpp std_msgs)
target_link_libraries(client_server ${cpp_typesupport_target})

add_executable(image_angle_change_service_server src/image_angle_change_service_server.cpp)
ament_target_dependencies(image_angle_change_service_server rclcpp std_msgs cv_bridge OpenCV sensor_msgs)
target_link_libraries(image_angle_change_service_server ${cpp_typesupport_target})

add_executable(image_angle_change_client_server src/image_angle_change_client_server.cpp)
ament_target_dependencies(image_angle_change_client_server rclcpp std_msgs cv_bridge OpenCV sensor_msgs)
target_link_libraries(image_angle_change_client_server ${cpp_typesupport_target})

install(TARGETS 
  rpm_publisher
  speed_publisher
  service_server
  client_server
  image_angle_change_service_server
  image_angle_change_client_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
