#include "subwindow_makeroute.h"

SubWindow_MakeRoute::SubWindow_MakeRoute(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_MakeRoute)
{
    ui->setupUi(this);

    ros_make_route_process = new QProcess(this);;
    ros_manual_process = new QProcess(this);

}

SubWindow_MakeRoute::~SubWindow_MakeRoute()
{
    Utils::terminate_process(ros_manual_process);
    Utils::terminate_process(ros_make_route_process);

    delete ui;
}

void SubWindow_MakeRoute::on_pushButton_StartRoute_toggled(bool checked)
{
    if(checked)
    {
        //startup to make map
        Utils::start_process(ros_make_route_process, "amr_ros", "om_navi_make_route.launch.py");
        Utils::start_process(ros_manual_process, "amr_ros", "om_manual.launch.py");

        ui->pushButton_StartRoute->setText("Press to OFF");
    }
    else
    {
        //end of make map
        Utils::terminate_process(ros_manual_process);
        Utils::terminate_process(ros_make_route_process);
        ui->pushButton_StartRoute->setText("Press to ON");
    }

}

