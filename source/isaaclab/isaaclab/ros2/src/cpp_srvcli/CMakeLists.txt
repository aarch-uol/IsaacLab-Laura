cmake_minimum_required(VERSION 3.8)
project(cpp_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) 

set(srv_files
  "srv/GetLatestMessage.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs # Your service uses std_msgs/String, so declare this dependency
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  # Include the directory for your generated service headers
  ${PROJECT_NAME}__rosidl_generator_cpp # This is the generated include path for your custom service
)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp std_msgs ${PROJECT_NAME}__rosidl_generator_cpp)



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

install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY share/${PROJECT_NAME}/srv/
  DESTINATION share/${PROJECT_NAME}/srv
)
ament_export_targets(server HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  rosidl_default_runtime # Essential for runtime use of generated interfaces
)



ament_package()
