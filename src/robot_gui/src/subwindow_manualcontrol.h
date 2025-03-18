#ifndef SUBWINDOW_MANUALCONTROL_H
#define SUBWINDOW_MANUALCONTROL_H
#include "ui_subwindow_manualcontrol.h"

#include <QDialog>
#include <QString>
#include <QProcess>

//ROS2 Files
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "om_cart/msg/cmd.hpp"

#include "utils.h"


namespace Ui {
class SubWindow_ManualControl;
}

class SubWindow_ManualControl : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_ManualControl(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SubWindow_ManualControl();

protected:
    void closeEvent(QCloseEvent *event) override
    {
        emit subWindowClosed();
        QDialog::closeEvent(event);
    }

signals:
    void subWindowClosed();

private slots:
    void on_pushButton_ManualStart_toggled(bool checked);

private:
    rclcpp::Node::SharedPtr node_;
    QProcess* ros_manual_process;

private:
    Ui::SubWindow_ManualControl *ui;
};

#endif // SUBWINDOW_MANUALCONTROL_H
