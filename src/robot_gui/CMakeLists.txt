cmake_minimum_required(VERSION 3.8)
project(robot_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ament_cmake 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geographic_msgs REQUIRED)

find_package(Bullet REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(om_cart REQUIRED)


# 查找 Qt5 5.15 的 Widgets 和 Multimedia 模块
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 列出所有 UI 文件
set(UI_FILES
  ui/mainwindow.ui
  ui/subwindow_device.ui
  ui/subwindow_system.ui
  ui/subwindow_manualcontrol.ui
  ui/subwindow_makemap.ui
  ui/subwindow_makeroute.ui
  ui/subwindow_guiderobot.ui
  ui/subwindow_geoservicetool.ui
  ui/subwindow_facelogin.ui
  ui/setup_init_location.ui

)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 使用 qt5_wrap_ui 将 UI 文件转换成对应的头文件
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${BULLET_INCLUDE_DIRS})
message("CMAKE_CURRENT_SOURCE_DIR:" ${CMAKE_CURRENT_SOURCE_DIR})


# 添加可执行文件，包含自动生成的 UI 头文件
add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/mainwindow.cpp
    src/subwindow_device.cpp
    src/subwindow_system.cpp
    src/subwindow_manualcontrol.cpp
    src/utils.cpp
    src/global_dataset.cpp
    src/subwindow_makemap.cpp
    src/subwindow_makeroute.cpp
    src/subwindow_guiderobot.cpp
    src/audiomanager.cpp
    src/subwindow_geoservicetool.cpp
    src/subwindow_facelogin.cpp
    src/setup_init_location.cpp
    ${UI_HEADERS})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geographic_msgs robot_localization
                            sensor_msgs geometry_msgs nav_msgs std_srvs tf2_ros tf2_eigen visualization_msgs om_cart)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)

# 链接 Qt 和 OpenCV 库
target_link_libraries(${PROJECT_NAME}_node Qt5::Widgets Qt5::Multimedia Qt5::MultimediaWidgets Qt5::Core ${OpenCV_LIBS} ${BULLET_LIBRARIES})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
