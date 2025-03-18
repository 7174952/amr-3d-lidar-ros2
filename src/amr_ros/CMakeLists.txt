cmake_minimum_required(VERSION 3.8)
project(amr_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ament_cmake 依赖
find_package(ament_cmake REQUIRED)

# 查找 Qt5 5.15 的 Widgets 和 Multimedia 模块
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Multimedia)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)

# 链接 Qt 和 OpenCV 库
target_link_libraries(my_node Qt5::Widgets Qt5::Multimedia Qt5::Core ${OpenCV_LIBS})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
