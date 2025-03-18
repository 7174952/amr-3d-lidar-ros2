#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setCentralWidget(ui->mdiArea);

    initConfig();

    Utils::execute_shell_cmd(sys_path["ScriptPath"] + "/init_robot.sh");
    robot_driver_process = new QProcess(this);
    Utils::start_process(robot_driver_process, "amr_ros", "robot_driver.launch.py");


    // 创建 ROS2 节点
    node_ = rclcpp::Node::make_shared("robot_gui_node");
    // 使用QTimer定时调用ROS2的spin_some()处理消息回调，避免阻塞GUI线程
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::spin_ros);
    timer_->start(10);  // 每10毫秒调用一次

    // 使用共享节点创建发布者，发布到 "cart_cmd" 话题
    cart_cmd_pub = node_->create_publisher<om_cart::msg::Cmd>("cart_cmd", 10);

}

MainWindow::~MainWindow()
{
    Utils::terminate_process(robot_driver_process);

    delete ui;
}

void MainWindow::initConfig()
{
    QSettings settings("mikuni", "GuideRobot/Path");

    sys_path["ProjectPath"]    = settings.value("ProjectPath", "").toString();
    sys_path["InstallPath"]    = settings.value("InstallPath", "").toString();
    sys_path["MapPath"]        = settings.value("MapPath", "").toString();
    sys_path["PointCloudPath"] = settings.value("PointCloudPath", "").toString();
    sys_path["BagDataPath"]    = settings.value("BagDataPath", "").toString();
    sys_path["ScriptPath"]     = settings.value("ScriptPath", "").toString();
    sys_path["LaunchPath"]     = settings.value("LaunchPath", "").toString();
    sys_path["ResourcePath"]   = settings.value("ResourcePath", "").toString();

}

void MainWindow::on_action_Device_Setup_triggered()
{
    SubWindow_Device subWin_device(this);
    subWin_device.exec();

}

void MainWindow::on_action_Manual_Control_triggered()
{
    SubWindow_ManualControl *subwin_manualControl = new SubWindow_ManualControl(node_);
    QMdiSubWindow *submdiwin_manual = ui->mdiArea->addSubWindow(subwin_manualControl);
    // submdiwin_manual->resize(800, 700);
    submdiwin_manual->setMaximumSize(800,700);
    submdiwin_manual->setWindowFlags(Qt::Dialog | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    // 连接自定义信号到主窗口的槽（例如 onDialogClosed）
    connect(subwin_manualControl, &SubWindow_ManualControl::subWindowClosed, this, &MainWindow::onManualControlClosed);

    submdiwin_manual->show();

    ui->action_Manual_Control->setDisabled(true);
}

void MainWindow::on_action_System_Setup_triggered()
{
    SubWindow_System subwin_SystemSetup(this);
    if(subwin_SystemSetup.exec() == QDialog::Accepted)
    {
        sys_path = subwin_SystemSetup.getSystemConfigPath();
    }
}

void MainWindow::onManualControlClosed()
{
    ui->action_Manual_Control->setEnabled(true);
}

void MainWindow::onMakeMapClosed()
{
    ui->action_Make_Map->setEnabled(true);
}

void MainWindow::onMakeRouteClosed()
{
    ui->action_Make_Route->setEnabled(true);
}


void MainWindow::on_action_Cart_status_toggled(bool arg1)
{
    ui->dockWidget_CartStatus->setVisible(arg1);
}


void MainWindow::on_action_Show_Config_toggled(bool arg1)
{
    ui->dockWidget_UsrConfig->setVisible(arg1);
}


void MainWindow::on_actionSet_Motor_Brake_toggled(bool arg1)
{
    cmd_msg.type = 1;
    cmd_msg.size = 1;
    cmd_msg.data[0] = arg1 ? 0 : 1; //0 - lock, 1 - unlock
    cart_cmd_pub->publish(cmd_msg);

    return;
}


void MainWindow::on_action_Make_Map_triggered()
{
    SubWindow_MakeMap *subwin_makeMap = new SubWindow_MakeMap(node_);
    QMdiSubWindow *submdiwin_makeMap = ui->mdiArea->addSubWindow(subwin_makeMap);
    submdiwin_makeMap->setMinimumSize(1200,720);
    submdiwin_makeMap->setMaximumSize(1200,720);
    submdiwin_makeMap->setWindowFlags(Qt::Dialog | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    // 连接自定义信号到主窗口的槽（例如 onDialogClosed）
    connect(subwin_makeMap, &SubWindow_MakeMap::subWindowClosed, this, &MainWindow::onMakeMapClosed);

    submdiwin_makeMap->show();

    ui->action_Make_Map->setDisabled(true);

}

void MainWindow::on_action_Make_Route_triggered()
{
    SubWindow_MakeRoute *subwin_makeRoute = new SubWindow_MakeRoute(node_);
    QMdiSubWindow *submdiwin_makeRoute = ui->mdiArea->addSubWindow(subwin_makeRoute);
    submdiwin_makeRoute->setMinimumSize(1200,720);
    submdiwin_makeRoute->setMaximumSize(1200,720);
    submdiwin_makeRoute->setWindowFlags(Qt::Dialog | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    // 连接自定义信号到主窗口的槽（例如 onDialogClosed）
    connect(subwin_makeRoute, &SubWindow_MakeRoute::subWindowClosed, this, &MainWindow::onMakeRouteClosed);

    submdiwin_makeRoute->show();

    ui->action_Make_Route->setDisabled(true);

}

