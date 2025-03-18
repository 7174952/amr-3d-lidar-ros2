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
    QSettings settings("mikuni", "GuideRobot/Path");

    system_path["ProjectPath"]    = settings.value("ProjectPath", "").toString();
    ui->lineEdit_ProjectPath->setText(system_path["ProjectPath"]);

    system_path["InstallPath"]    = settings.value("InstallPath", "").toString();
    ui->lineEdit_InstallPath->setText(system_path["InstallPath"]);

    system_path["MapPath"]        = settings.value("MapPath", "").toString();
    ui->lineEdit_MapPath->setText(system_path["MapPath"]);

    system_path["PointCloudPath"] = settings.value("PointCloudPath", "").toString();
    ui->lineEdit_PointCloudPath->setText(system_path["PointCloudPath"]);

    system_path["BagDataPath"]    = settings.value("BagDataPath", "").toString();
    ui->lineEdit_BagDataPath->setText(system_path["BagDataPath"]);

    system_path["ScriptPath"]     = settings.value("ScriptPath", "").toString();
    ui->lineEdit_ScriptPath->setText(system_path["ScriptPath"]);

    system_path["LaunchPath"]     = settings.value("LaunchPath", "").toString();
    ui->lineEdit_LaunchPath->setText(system_path["LaunchPath"]);

    system_path["ResourcePath"]   = settings.value("ResourcePath", "").toString();
    ui->lineEdit_ResourcePath->setText(system_path["ResourcePath"]);


}

void SubWindow_System::saveConfig()
{
     QSettings settings("mikuni", "GuideRobot/Path");

     //Save all path
     settings.setValue("ProjectPath", system_path["ProjectPath"]);
     settings.setValue("InstallPath", system_path["InstallPath"]);
     settings.setValue("MapPath", system_path["MapPath"]);
     settings.setValue("PointCloudPath", system_path["PointCloudPath"]);
     settings.setValue("BagDataPath", system_path["BagDataPath"]);
     settings.setValue("ScriptPath", system_path["ScriptPath"]);
     settings.setValue("LaunchPath", system_path["LaunchPath"]);
     settings.setValue("ResourcePath", system_path["ResourcePath"]);

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

QMap<QString, QString> SubWindow_System::getSystemConfigPath()
{
    return system_path;
}

void SubWindow_System::on_pushButton_ProjectPath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_ProjectPath->setText(dir);
        system_path["ProjectPath"] = dir;

    }
}


void SubWindow_System::on_pushButton_InstallPath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_InstallPath->setText(dir);
        system_path["InstallPath"] = dir;
    }
}


void SubWindow_System::on_pushButton_MapPath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_MapPath->setText(dir);
        system_path["MapPath"] = dir;

    }

}


void SubWindow_System::on_pushButton_PointCloudPath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_PointCloudPath->setText(dir);
        system_path["PointCloudPath"] = dir;
    }

}


void SubWindow_System::on_pushButton_BagDataPath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_BagDataPath->setText(dir);
        system_path["BagDataPath"] = dir;

    }

}


void SubWindow_System::on_pushButton_ScriptPath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_ScriptPath->setText(dir);
        system_path["ScriptPath"] = dir;

    }

}


void SubWindow_System::on_pushButton_LaunchPath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_LaunchPath->setText(dir);
        system_path["LaunchPath"] = dir;

    }

}


void SubWindow_System::on_pushButton_ResourcePath_clicked()
{
    QString dir = setup_path();

    if (!dir.isEmpty())
    {
        ui->lineEdit_ResourcePath->setText(dir);
        system_path["ResourcePath"] = dir;

    }

}


void SubWindow_System::on_buttonBox_accepted()
{
    saveConfig();
}

