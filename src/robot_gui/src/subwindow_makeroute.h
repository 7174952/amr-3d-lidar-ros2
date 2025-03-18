#ifndef SUBWINDOW_MAKEROUTE_H
#define SUBWINDOW_MAKEROUTE_H

#include "ui_subwindow_makeroute.h"

#include <QDialog>

//ROS2 Files
#include <rclcpp/rclcpp.hpp>

#include "utils.h"


namespace Ui {
class SubWindow_MakeRoute;
}

class SubWindow_MakeRoute : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_MakeRoute(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SubWindow_MakeRoute();

protected:
    void closeEvent(QCloseEvent *event) override
    {
        emit subWindowClosed();
        QDialog::closeEvent(event);
    }

signals:
    void subWindowClosed();

private slots:
    void on_pushButton_StartRoute_toggled(bool checked);

private:
    rclcpp::Node::SharedPtr node_;
    QProcess* ros_make_route_process;
    QProcess* ros_manual_process;


private:
    Ui::SubWindow_MakeRoute *ui;
};

#endif // SUBWINDOW_MAKEROUTE_H
