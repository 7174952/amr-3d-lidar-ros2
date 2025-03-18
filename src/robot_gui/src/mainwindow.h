#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"

#include <QMainWindow>
#include <QToolBar>
#include <QStatusBar>
#include <QAction>
#include <QLabel>
#include <QMdiSubWindow>
#include <QMap>
#include <QTimer>
#include <QSettings>

#include <QDebug>

#include "subwindow_device.h"
#include "subwindow_system.h"
#include "subwindow_manualcontrol.h"
#include "subwindow_makemap.h"
#include "subwindow_makeroute.h"
#include "utils.h"

#include <rclcpp/rclcpp.hpp>

#include "om_cart/msg/cmd.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private:
    void initConfig();

private slots:
    void onManualControlClosed();
    void onMakeMapClosed();
    void onMakeRouteClosed();

    // 定时调用，用于处理ROS2消息回调
    void spin_ros()
    {
        rclcpp::spin_some(node_);
    }

  void on_action_Device_Setup_triggered();

  void on_action_Manual_Control_triggered();

  void on_action_System_Setup_triggered();

  void on_action_Cart_status_toggled(bool arg1);

  void on_action_Show_Config_toggled(bool arg1);

  void on_actionSet_Motor_Brake_toggled(bool arg1);

  void on_action_Make_Map_triggered();

  void on_action_Make_Route_triggered();

private:
   rclcpp::Node::SharedPtr node_;
   rclcpp::Publisher<om_cart::msg::Cmd>::SharedPtr cart_cmd_pub;
   om_cart::msg::Cmd cmd_msg;


   QTimer* timer_;

   const QStringList PATH_LIST = {"ProjectPath",
                              "InstallPath",
                              "MapPath",
                              "PointCloudPath",
                              "BagDataPath",
                              "ScriptPath",
                              "LaunchPath",
                              "ResourcePath"};

private:
  Ui::MainWindow *ui;
  QMap<QString, QString> sys_path;

  QProcess* robot_driver_process;


};

#endif // MAINWINDOW_H
