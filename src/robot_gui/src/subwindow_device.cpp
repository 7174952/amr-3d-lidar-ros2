#include "subwindow_device.h"
#include "ui_subwindow_device.h"

SubWindow_Device::SubWindow_Device(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SubWindow_Device)
{
    ui->setupUi(this);
}

SubWindow_Device::~SubWindow_Device()
{
    delete ui;
}
