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
#include <QCloseEvent>
#include <QDir>

#include <QDebug>

#include "subwindow_device.h"
#include "subwindow_system.h"
#include "subwindow_manualcontrol.h"
#include "subwindow_makemap.h"
#include "subwindow_makeroute.h"
#include "subwindow_guiderobot.h"

#include "utils.h"
#include "global_dataset.h"

#include <rclcpp/rclcpp.hpp>

#include "om_cart/msg/cmd.hpp"
#include "om_cart/msg/status.hpp"

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
    void saveUsrConfig();

signals:
    void mapMapNameChanged(const QString& newMapName);

private slots:
      void showMessageInStatusBar(const QString &message);
      void handleOutput();
      void handleError();

private slots:
    void onManualControlClosed();
    void onMakeMapClosed();
    void onMakeRouteClosed();
    void onGuideRobotClosed();
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

  void on_comboBox_MapFolder_currentTextChanged(const QString &arg1);

  void on_actionInit_Pose_triggered();

  void on_action_Guide_Robot_triggered();

  void on_action_Debug_Enable_triggered(bool checked);

  void on_checkBox_GnssSensor_stateChanged(int arg1);

private:
  void cartStatus_CallBack(const om_cart::msg::Status& status);


private:
   rclcpp::Node::SharedPtr node_;
   rclcpp::Publisher<om_cart::msg::Cmd>::SharedPtr cart_cmd_pub;
   om_cart::msg::Cmd cmd_msg;

   rclcpp::Subscription<om_cart::msg::Status>::SharedPtr cart_status_sub;

   QTimer* timer_;

   enum
   {
       CART_STATUS = 1,
       COMM_RESULT = 2,
   };

   typedef union
   {
        struct
        {
           double vel_line;              //0
           double vel_theta;             //1
           double vel_left;              //2
           double vel_right;             //3
           double alm_code_L;            //4
           double alm_code_R;            //5
           double main_power_volt_L;     //6
           double main_power_volt_R;     //7
           double main_power_curr_L;     //8
           double main_power_curr_R;     //9
           double motor_temp_L;          //10
           double motor_temp_R;          //11
           double driver_temp_L;         //12
           double driver_temp_R;         //13
           double emergen_stop;          //14
        } cart_status;

        struct
        {
           double result;
           double error_code;
        }  comm_status;

        double data[30];

   } Cart_Status_Info;
   const double CART_BAT_MAX_VOLT = 25.4; //Volt
   const double CART_BAT_MIN_VOLT = 21.0; //Volt

private:
  Ui::MainWindow *ui;

  QProcess* robot_driver_process;
  QProcess gnss_rtk_process;

};

#endif // MAINWINDOW_H
