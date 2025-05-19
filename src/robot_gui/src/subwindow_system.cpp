#include "subwindow_system.h"
#include "ui_subwindow_system.h"

SubWindow_System::SubWindow_System(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SubWindow_System)
{
    ui->setupUi(this);
    initConfig();
}

SubWindow_System::~SubWindow_System()
{
    delete ui;
}

void SubWindow_System::initConfig()
{
    system_path = Global_DataSet::instance().sysPath();
    ui->lineEdit_ProjectPath->setText(system_path["ProjectPath"]);
    ui->lineEdit_InstallPath->setText(system_path["InstallPath"]);
    ui->lineEdit_MapPath->setText(system_path["MapPath"]);
    ui->lineEdit_PointCloudPath->setText(system_path["PointCloudPath"]);
    ui->lineEdit_BagDataPath->setText(system_path["BagDataPath"]);
    ui->lineEdit_ScriptPath->setText(system_path["ScriptPath"]);
    ui->lineEdit_LaunchPath->setText(system_path["LaunchPath"]);
    ui->lineEdit_ResourcePath->setText(system_path["ResourcePath"]);
    ui->lineEdit_PythonPathCamera->setText(system_path["PyPathCamera"]);
    ui->lineEdit_PythonPathVoice->setText(system_path["PyPathVoice"]);

    robot_size = Global_DataSet::instance().robotSize();
    ui->lineEdit_obstRangeMin->setText(robot_size["ObstRangeMin"]);
    ui->lineEdit_obstRangeMax->setText(robot_size["ObstRangeMax"]);
    ui->lineEdit_robotWidth->setText(robot_size["RobotWidth"]);
    ui->lineEdit_robotHeight->setText(robot_size["RobotHeight"]);
    ui->lineEdit_obstTolerance->setText(robot_size["RobotWidthTole"]);
    ui->lineEdit_distanceLidarToGnss->setText(robot_size["DistanceLidarToGnss"]);

    gnss_ntrip = Global_DataSet::instance().gnssNtrip();
    ui->comboBox_ntripSite->setCurrentText(gnss_ntrip["NtripSite"]);
    ui->comboBox_accessPort->setCurrentText(gnss_ntrip["NtripPort"]);
    ui->comboBox_mountPoint->setCurrentText(gnss_ntrip["NtripMountPoint"]);
    ui->lineEdit_password->setText(gnss_ntrip["NtripPassword"]);
    ui->lineEdit_convThreshold->setText(gnss_ntrip["ConvThreshold"]);
}

void SubWindow_System::saveConfig()
{
     QSettings settings("mikuni", "GuideRobot/Path");

     system_path["ProjectPath"] = ui->lineEdit_ProjectPath->text();
     system_path["InstallPath"] = ui->lineEdit_InstallPath->text();
     system_path["MapPath"] = ui->lineEdit_MapPath->text();
     system_path["PointCloudPath"] = ui->lineEdit_PointCloudPath->text();
     system_path["BagDataPath"] = ui->lineEdit_BagDataPath->text();
     system_path["ScriptPath"] =  ui->lineEdit_ScriptPath->text();
     system_path["LaunchPath"] = ui->lineEdit_LaunchPath->text();
     system_path["ResourcePath"] = ui->lineEdit_ResourcePath->text();
     system_path["PyPathCamera"] = ui->lineEdit_PythonPathCamera->text();
     system_path["PyPathVoice"] = ui->lineEdit_PythonPathVoice->text();

     //Save all path
     settings.setValue("ProjectPath", system_path["ProjectPath"]);
     settings.setValue("InstallPath", system_path["InstallPath"]);
     settings.setValue("MapPath", system_path["MapPath"]);
     settings.setValue("PointCloudPath", system_path["PointCloudPath"]);
     settings.setValue("BagDataPath", system_path["BagDataPath"]);
     settings.setValue("ScriptPath", system_path["ScriptPath"]);
     settings.setValue("LaunchPath", system_path["LaunchPath"]);
     settings.setValue("ResourcePath", system_path["ResourcePath"]);
     settings.setValue("PyPathCamera", system_path["PyPathCamera"]);
     settings.setValue("PyPathVoice", system_path["PyPathVoice"]);


     Global_DataSet::instance().setSysPath(system_path);

     //save all robot size
     QSettings settings_size("mikuni", "GuideRobot/Size");
     robot_size["ObstRangeMin"] = ui->lineEdit_obstRangeMin->text();
     robot_size["ObstRangeMax"] = ui->lineEdit_obstRangeMax->text();
     robot_size["RobotWidth"] = ui->lineEdit_robotWidth->text();
     robot_size["RobotHeight"] = ui->lineEdit_robotHeight->text();
     robot_size["RobotWidthTole"] = ui->lineEdit_obstTolerance->text();
     robot_size["DistanceLidarToGnss"] = ui->lineEdit_distanceLidarToGnss->text();

     settings_size.setValue("ObstRangeMin", robot_size["ObstRangeMin"]);
     settings_size.setValue("ObstRangeMax", robot_size["ObstRangeMax"]);
     settings_size.setValue("RobotWidth", robot_size["RobotWidth"]);
     settings_size.setValue("RobotHeight", robot_size["RobotHeight"]);
     settings_size.setValue("RobotWidthTole", robot_size["RobotWidthTole"]);
     settings_size.setValue("DistanceLidarToGnss", robot_size["DistanceLidarToGnss"]);
     Global_DataSet::instance().setRobotSize(robot_size);

     //save all ntrip info
     QSettings settings_ntrip("mikuni", "GuideRobot/ntrip");
     gnss_ntrip["NtripSite"] = ui->comboBox_ntripSite->currentText();
     gnss_ntrip["NtripPort"] = ui->comboBox_accessPort->currentText();
     gnss_ntrip["NtripMountPoint"] = ui->comboBox_mountPoint->currentText();
     gnss_ntrip["NtripPassword"] = ui->lineEdit_password->text();
     gnss_ntrip["ConvThreshold"] = ui->lineEdit_convThreshold->text();

     settings_ntrip.setValue("NtripSite", gnss_ntrip["NtripSite"]);
     settings_ntrip.setValue("NtripPort", gnss_ntrip["NtripPort"]);
     settings_ntrip.setValue("NtripMountPoint", gnss_ntrip["NtripMountPoint"]);
     settings_ntrip.setValue("NtripPassword", gnss_ntrip["NtripPassword"]);
     settings_ntrip.setValue("ConvThreshold", gnss_ntrip["ConvThreshold"]);

     Global_DataSet::instance().setGnssNtrip(gnss_ntrip);


}

QString SubWindow_System::setup_path()
{
    QString dir = QFileDialog::getExistingDirectory(
        this,
        tr("Please Select Folder"),
        QDir::homePath(), // 默认打开的路径
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    return dir;
}

void SubWindow_System::on_pushButton_ProjectPath_clicked()
{
    ui->lineEdit_ProjectPath->setText(setup_path());

}

void SubWindow_System::on_pushButton_InstallPath_clicked()
{
    ui->lineEdit_InstallPath->setText(setup_path());
}

void SubWindow_System::on_pushButton_MapPath_clicked()
{
    ui->lineEdit_MapPath->setText(setup_path());
}

void SubWindow_System::on_pushButton_PointCloudPath_clicked()
{
    ui->lineEdit_PointCloudPath->setText(setup_path());
}

void SubWindow_System::on_pushButton_BagDataPath_clicked()
{
    ui->lineEdit_BagDataPath->setText(setup_path());
}

void SubWindow_System::on_pushButton_ScriptPath_clicked()
{
    ui->lineEdit_ScriptPath->setText(setup_path());
}

void SubWindow_System::on_pushButton_LaunchPath_clicked()
{
    ui->lineEdit_LaunchPath->setText(setup_path());
}

void SubWindow_System::on_pushButton_ResourcePath_clicked()
{
    ui->lineEdit_ResourcePath->setText(setup_path());
}

void SubWindow_System::on_buttonBox_accepted()
{
    saveConfig();
}


void SubWindow_System::on_pushButton_PythonPathCamera_clicked()
{
    ui->lineEdit_PythonPathCamera->setText(setup_path());

}


void SubWindow_System::on_pushButton_PythonPathVoice_clicked()
{
    ui->lineEdit_PythonPathVoice->setText(setup_path());

}

