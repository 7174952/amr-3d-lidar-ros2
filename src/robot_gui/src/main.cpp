#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "mainwindow.h"      // 对应 MainWindow.ui 的生成窗口类
//#include "SettingsWindow.hpp"  // 对应 SettingsWindow.ui 的生成窗口类

int main(int argc, char *argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 初始化 Qt 应用程序
  QApplication app(argc, argv);

  // 创建各个窗口（根据需要选择显示哪些）
  MainWindow mainWindow;
  mainWindow.show();

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
