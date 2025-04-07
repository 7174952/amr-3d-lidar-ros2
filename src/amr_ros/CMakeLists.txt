cmake_minimum_required(VERSION 3.8)
project(amr_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ament_cmake 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)

# 查找 Qt5 5.15 的 Widgets 和 Multimedia 模块
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Multimedia)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)

add_executable(obstacle_detector_node src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector_node rclcpp std_msgs sensor_msgs geometry_msgs pcl_conversions)
target_link_libraries(obstacle_detector_node ${PCL_LIBRARIES})

#target_link_libraries(obstacle_detector_node Qt5::Widgets Qt5::Multimedia Qt5::Core ${OpenCV_LIBS} ${PCL_LIBRARIES})

install(TARGETS
    obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
