cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(orocos_kdl REQUIRED)

# 添加可执行文件
add_executable(pure_pursuit src/pure_pursuit.cpp)

# 链接依赖库
ament_target_dependencies(pure_pursuit
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  visualization_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(pure_pursuit
    ${orocos_kdl_LIBRARIES}

)

# 安装可执行文件
install(TARGETS
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
