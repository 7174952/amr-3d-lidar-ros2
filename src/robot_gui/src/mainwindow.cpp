#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setCentralWidget(ui->mdiArea);

    initConfig();

     QMap<QString, QString> sys_path = Global_DataSet::instance().sysPath();

    Utils::execute_shell_cmd(sys_path["ScriptPath"] + "/init_robot.sh");
    robot_driver_process = new QProcess(this);
    Utils::start_process(robot_driver_process, "amr_ros", "robot_driver.launch.py");
    Global_DataSet::instance().setDebugMode(false);

    // 创建 ROS2 节点
    node_ = rclcpp::Node::make_shared("robot_gui_node");
    // 使用QTimer定时调用ROS2的spin_some()处理消息回调，避免阻塞GUI线程
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::spin_ros);
    timer_->start(10);  // 每10毫秒调用一次

    // 使用共享节点创建发布者，发布到 "cart_cmd" 话题
    cart_cmd_pub = node_->create_publisher<om_cart::msg::Cmd>("cart_cmd", 10);
    cart_status_sub = node_->create_subscription<om_cart::msg::Status>("cart_status", 10, std::bind(&MainWindow::cartStatus_CallBack, this, std::placeholders::_1));

    QObject::connect(ui->action_Exit, &QAction::triggered, this, &QApplication::quit);

    ui->progressBar_BatPercent->setValue(0);
    ui->lineEdit_BatVolt->setText("0.0");
    ui->lineEdit_BatCurr->setText("0.0");
    ui->lineEdit_LineSpeed->setText("0.0");

    // show gnss ntrip status
    connect(&gnss_rtk_process, &QProcess::readyReadStandardOutput,
            this,  &MainWindow::handleOutput);
    connect(&gnss_rtk_process, &QProcess::errorOccurred,
            this,  &MainWindow::handleError);

}

MainWindow::~MainWindow()
{
    saveUsrConfig();
    Utils::terminate_process(robot_driver_process);

    delete ui;
}

void MainWindow::initConfig()
{        
    QSettings settings("mikuni", "GuideRobot/Path");

    QMap<QString, QString> sys_path;

    sys_path["ProjectPath"]    = settings.value("ProjectPath", "").toString();
    sys_path["InstallPath"]    = settings.value("InstallPath", "").toString();
    sys_path["MapPath"]        = settings.value("MapPath", "").toString();
    sys_path["PointCloudPath"] = settings.value("PointCloudPath", "").toString();
    sys_path["BagDataPath"]    = settings.value("BagDataPath", "").toString();
    sys_path["ScriptPath"]     = settings.value("ScriptPath", "").toString();
    sys_path["LaunchPath"]     = settings.value("LaunchPath", "").toString();
    sys_path["ResourcePath"]   = settings.value("ResourcePath", "").toString();
    sys_path["PyPathCamera"]   = settings.value("PyPathCamera", "").toString();
    sys_path["PyPathVoice"]    = settings.value("PyPathVoice", "").toString();

    Global_DataSet::instance().setSysPath(sys_path);

    QDir dir(sys_path["MapPath"]);
    QStringList subdirs = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    ui->comboBox_MapFolder->addItems(subdirs);

    QSettings settings_ntrip("mikuni", "GuideRobot/ntrip");
    QMap<QString, QString> gnss_ntrip;
    gnss_ntrip["NtripSite"] = settings_ntrip.value("NtripSite", "").toString();
    gnss_ntrip["NtripPort"] = settings_ntrip.value("NtripPort", "2101").toString();
    gnss_ntrip["NtripMountPoint"] = settings_ntrip.value("NtripMountPoint", "").toString();
    gnss_ntrip["NtripPassword"] = settings_ntrip.value("NtripPassword", "").toString();
    Global_DataSet::instance().setGnssNtrip(gnss_ntrip);

    QSettings settings_navi("mikuni", "GuideRobot/Navi");
    QString curr_mapName = settings_navi.value("CurrentMapName", "").toString();
    ui->comboBox_MapFolder->setCurrentText(curr_mapName);
    Global_DataSet::instance().setCurrentMapName(curr_mapName);
    //Load sensor configurations
    bool front_camera_enable = settings_navi.value("FrontCameraEn", "false").toBool();
    ui->checkBox_FrontCamera->setChecked(front_camera_enable);
    Global_DataSet::instance().setSensorEnable("FrontCameraEn",front_camera_enable);

    bool guide_camera_enable = settings_navi.value("GuideCameraEn", "false").toBool();
    ui->checkBox_GuideCamera->setChecked(guide_camera_enable);
    Global_DataSet::instance().setSensorEnable("GuideCameraEn",guide_camera_enable);

    bool gnss_enable = settings_navi.value("GnssEn", "false").toBool();
    ui->checkBox_GnssSensor->setChecked(gnss_enable);
    Global_DataSet::instance().setSensorEnable("GnssEn", gnss_enable);

    QSettings settings_size("mikuni", "GuideRobot/Size");
    QMap<QString, QString> robot_size;
    robot_size["ObstRangeMin"] = settings_size.value("ObstRangeMin", "0.0").toString();
    robot_size["ObstRangeMax"] = settings_size.value("ObstRangeMax", "1.0").toString();
    robot_size["RobotWidth"] = settings_size.value("RobotWidth", "").toString();
    robot_size["RobotHeight"] = settings_size.value("RobotHeight", "").toString();
    robot_size["RobotWidthTole"] = settings_size.value("RobotWidthTole", "0.05").toString();
    Global_DataSet::instance().setRobotSize(robot_size);

}

void MainWindow::saveUsrConfig()
{
    QSettings settings_navi("mikuni", "GuideRobot/Navi");

    settings_navi.setValue("CurrentMapName", ui->comboBox_MapFolder->currentText());
    settings_navi.setValue("FrontCameraEn", ui->checkBox_FrontCamera->isChecked());
    settings_navi.setValue("GuideCameraEn", ui->checkBox_GuideCamera->isChecked());
    settings_navi.setValue("GnssEn", ui->checkBox_GnssSensor->isChecked());

}

void MainWindow::on_action_Device_Setup_triggered()
{
    SubWindow_Device subWin_device(this);
    subWin_device.exec();

}

void MainWindow::showMessageInStatusBar(const QString &message)
{
    ui->statusBar_main->showMessage(message,20000);
}

void MainWindow::cartStatus_CallBack(const om_cart::msg::Status& status)
{
    if(status.type != CART_STATUS)
    {
        return;
    }

    Cart_Status_Info status_info;
    for (int i = 0; i < status.size; ++i)
    {
        status_info.data[i] = status.data[i];
    }
    double tmp_val = (status_info.cart_status.main_power_volt_L/10 + status_info.cart_status.main_power_volt_R/10)/2;
    double volt_rate = ((tmp_val - CART_BAT_MIN_VOLT) / (CART_BAT_MAX_VOLT - CART_BAT_MIN_VOLT)) * 100;
    ui->progressBar_BatPercent->setValue(volt_rate);
    ui->lineEdit_BatVolt->setText(QString::number(tmp_val,'f',1));

    tmp_val = (status_info.cart_status.main_power_curr_L + status_info.cart_status.main_power_curr_R)/2;
    ui->lineEdit_BatCurr->setText(QString::number(tmp_val/1000,'f',1));

    tmp_val = status_info.cart_status.vel_line;
    if(ui->comboBox_SpeedUnit->currentText().contains("m/s"))
    {
        ui->lineEdit_LineSpeed->setText(QString::number(tmp_val,'f',2));
    }
    else //km/h
    {
        ui->lineEdit_LineSpeed->setText(QString::number(tmp_val * 3.6,'f',2));
    }
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

    ui->action_Manual_Control->setEnabled(false);
}

void MainWindow::on_action_System_Setup_triggered()
{
    SubWindow_System subwin_SystemSetup(this);
    if(subwin_SystemSetup.exec() == QDialog::Accepted)
    {

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
    ui->comboBox_MapFolder->setEnabled(true);
}

void MainWindow::onGuideRobotClosed()
{
    ui->action_Guide_Robot->setEnabled(true);
    ui->comboBox_MapFolder->setEnabled(true);
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
    submdiwin_makeMap->setMinimumSize(1200,800);
    submdiwin_makeMap->setMaximumSize(1200,800);
    submdiwin_makeMap->setWindowFlags(Qt::Dialog | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    // 连接自定义信号到主窗口的槽（例如 onDialogClosed）
    connect(subwin_makeMap, &SubWindow_MakeMap::subWindowClosed, this, &MainWindow::onMakeMapClosed);
    connect(this, &MainWindow::mapMapNameChanged, subwin_makeMap, &SubWindow_MakeMap::updateMapName);
    emit mapMapNameChanged(ui->comboBox_MapFolder->currentText());

    submdiwin_makeMap->setWindowTitle("Mapping");
    submdiwin_makeMap->show();
    ui->action_Make_Map->setEnabled(false);
}

void MainWindow::on_action_Make_Route_triggered()
{
    SubWindow_MakeRoute *subwin_makeRoute = new SubWindow_MakeRoute(node_);
    QMdiSubWindow *submdiwin_makeRoute = ui->mdiArea->addSubWindow(subwin_makeRoute);
    submdiwin_makeRoute->setMinimumSize(1200,800);
    submdiwin_makeRoute->setMaximumSize(1200,800);
    submdiwin_makeRoute->setWindowFlags(Qt::Dialog | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    // 连接自定义信号到主窗口的槽（例如 onDialogClosed）
    connect(subwin_makeRoute, &SubWindow_MakeRoute::subWindowClosed, this, &MainWindow::onMakeRouteClosed);
    connect(this, &MainWindow::mapMapNameChanged, subwin_makeRoute, &SubWindow_MakeRoute::updateMapName);
    emit mapMapNameChanged(ui->comboBox_MapFolder->currentText());

    submdiwin_makeRoute->setWindowTitle("Make Route");
    submdiwin_makeRoute->show();
    ui->action_Make_Route->setEnabled(false);
    ui->comboBox_MapFolder->setEnabled(false);
}

void MainWindow::on_comboBox_MapFolder_currentTextChanged(const QString &arg1)
{
    emit mapMapNameChanged(arg1);
}

void MainWindow::on_actionInit_Pose_triggered()
{
    //call init_robot_pose.py
}

void MainWindow::on_action_Guide_Robot_triggered()
{
    SubWindow_GuideRobot *subwin_guideRobot = new SubWindow_GuideRobot(node_);

    QMdiSubWindow *submdiwin_guideRobot = ui->mdiArea->addSubWindow(subwin_guideRobot);
    submdiwin_guideRobot->setMinimumSize(1200,800);
    submdiwin_guideRobot->setMaximumSize(1200,800);
    submdiwin_guideRobot->setWindowFlags(Qt::Dialog | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    // 连接自定义信号到主窗口的槽（例如 onDialogClosed）
    connect(subwin_guideRobot, &SubWindow_GuideRobot::subWindowClosed, this, &MainWindow::onGuideRobotClosed);
    connect(this, &MainWindow::mapMapNameChanged, subwin_guideRobot, &SubWindow_GuideRobot::updateMapName);
    connect(subwin_guideRobot, &SubWindow_GuideRobot::sendMessage, this, &MainWindow::showMessageInStatusBar);
    connect(ui->checkBox_GuideCamera, &QCheckBox::stateChanged, subwin_guideRobot, &SubWindow_GuideRobot::onGuideCameraStateChanged);

    emit mapMapNameChanged(ui->comboBox_MapFolder->currentText());

    submdiwin_guideRobot->setWindowTitle("Guide Robot Operation");
    submdiwin_guideRobot->show();
    ui->action_Guide_Robot->setEnabled(false);
    ui->comboBox_MapFolder->setEnabled(false);

}

void MainWindow::on_action_Debug_Enable_triggered(bool checked)
{
    Global_DataSet::instance().setDebugMode(checked);
}

void MainWindow::handleOutput()
{
    QString output = gnss_rtk_process.readAllStandardOutput();
    qDebug() << output;
    ui->statusBar_main->showMessage(output);
}

void MainWindow::handleError()
{
    QString error = gnss_rtk_process.readAllStandardError();
    qDebug() << error;
    ui->statusBar_main->showMessage(error);
}

void MainWindow::on_checkBox_GnssSensor_stateChanged(int arg1)
{
    bool checked = (arg1 == 2) ? true:false;
    Global_DataSet::instance().setSensorEnable("GnssEn", checked);

    if(checked)
    {
        QString ntripSite = Global_DataSet::instance().gnssNtrip()["NtripSite"];
        QString ntripPort = Global_DataSet::instance().gnssNtrip()["NtripPort"];
        QString ntripMountPoint = Global_DataSet::instance().gnssNtrip()["NtripMountPoint"].split("-").at(0);
        QString ntripPassword = Global_DataSet::instance().gnssNtrip()["NtripPassword"];

        gnss_rtk_process.setProgram("str2str");
        if(ntripSite.contains("ntrip1"))
        {
            gnss_rtk_process.setArguments({
                "-in",  "ntrip://" + ntripSite + ":" + ntripPort + "/" + ntripMountPoint,
                "-out", "serial://ttyACM-gnss:38400:8:n:1"
            });
        }
        else
        {
            gnss_rtk_process.setArguments({
                "-in",  "ntrip://user@gmail.com:" + ntripPassword + "@" + ntripSite + ":" + ntripPort + "/" + ntripMountPoint,
                "-out", "serial://ttyACM-gnss:38400:8:n:1"
            });
        }

        // 把 stdout / stderr 合并，便于一次性读取
        gnss_rtk_process.setProcessChannelMode(QProcess::MergedChannels);

        gnss_rtk_process.start();
    }
    else
    {
        if(gnss_rtk_process.state() != QProcess::NotRunning)
            gnss_rtk_process.terminate();
    }

}

