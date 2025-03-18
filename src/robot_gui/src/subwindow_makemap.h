#ifndef SUBWINDOW_MAKEMAP_H
#define SUBWINDOW_MAKEMAP_H

#include "ui_subwindow_makemap.h"

#include <QDialog>

//ROS2 Files
#include <rclcpp/rclcpp.hpp>

#include "utils.h"


namespace Ui {
class SubWindow_MakeMap;
}

class SubWindow_MakeMap : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_MakeMap(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SubWindow_MakeMap();

protected:
    void closeEvent(QCloseEvent *event) override
    {
        emit subWindowClosed();
        QDialog::closeEvent(event);
    }

signals:
    void subWindowClosed();

private slots:
    void on_pushButton_StartMap_toggled(bool checked);

private:
    rclcpp::Node::SharedPtr node_;
    QProcess* ros_make_map_process;
    QProcess* ros_manual_process;



private:
    Ui::SubWindow_MakeMap *ui;
};

#endif // SUBWINDOW_MAKEMAP_H
