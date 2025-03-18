#include "subwindow_manualcontrol.h"

SubWindow_ManualControl::SubWindow_ManualControl(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_ManualControl)
{
    ui->setupUi(this);

    ros_manual_process = new QProcess(this);

}

SubWindow_ManualControl::~SubWindow_ManualControl()
{
    //end of cart and joy
    Utils::terminate_process(ros_manual_process);

    delete ui;
}

void SubWindow_ManualControl::on_pushButton_ManualStart_toggled(bool checked)
{
    if(checked)
    {
        //startup cart and joy
        Utils::start_process(ros_manual_process, "amr_ros", "om_manual.launch.py");
        ui->pushButton_ManualStart->setText("Press to OFF");
    }
    else
    {
        //end of cart and joy
        Utils::terminate_process(ros_manual_process);
        ui->pushButton_ManualStart->setText("Press to ON");
    }

}

