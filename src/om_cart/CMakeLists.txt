cmake_minimum_required(VERSION 3.8)
project(om_cart)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(om_msgs REQUIRED)
find_package(om_modbus_master REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  "msg/Cmd.msg"
  "msg/Status.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

include_directories(
  include
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Add executables
add_executable(om_cart_node src/om_cart.cpp src/utils.cpp)
ament_target_dependencies(om_cart_node rclcpp sensor_msgs om_modbus_master om_msgs)
target_link_libraries(om_cart_node "${cpp_typesupport_target}")

add_executable(om_manual_node src/om_manual.cpp)
ament_target_dependencies(om_manual_node rclcpp sensor_msgs om_modbus_master om_msgs)
target_link_libraries(om_manual_node "${cpp_typesupport_target}")


install(TARGETS om_cart_node om_manual_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
