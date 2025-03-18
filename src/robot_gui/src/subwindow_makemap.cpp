#include "subwindow_makemap.h"

SubWindow_MakeMap::SubWindow_MakeMap(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_MakeMap)
{
    ui->setupUi(this);

    ros_make_map_process = new QProcess(this);;
    ros_manual_process = new QProcess(this);

}

SubWindow_MakeMap::~SubWindow_MakeMap()
{
    Utils::terminate_process(ros_manual_process);
    Utils::terminate_process(ros_make_map_process);
    delete ui;
}

void SubWindow_MakeMap::on_pushButton_StartMap_toggled(bool checked)
{
    if(checked)
    {
        //startup to make map
        Utils::start_process(ros_make_map_process, "amr_ros", "om_map_liosam.launch.py");
        Utils::start_process(ros_manual_process, "amr_ros", "om_manual.launch.py");

        ui->pushButton_StartMap->setText("Press to OFF");
    }
    else
    {
        //end of make map
        Utils::terminate_process(ros_manual_process);
        Utils::terminate_process(ros_make_map_process);
        ui->pushButton_StartMap->setText("Press to ON");
    }

}

