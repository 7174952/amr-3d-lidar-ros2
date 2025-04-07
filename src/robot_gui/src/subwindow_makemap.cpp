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

void SubWindow_MakeMap::updateMapName(const QString& newMapName)
{
    m_mapName = newMapName;
}

void SubWindow_MakeMap::on_pushButton_StartMap_toggled(bool checked)
{
    if(checked)
    {
        if(m_mapName.isEmpty())
        {
            QMessageBox::warning(this,"Warning","Map Name is blank.Please Set the Map Name.");
            return;
        }

        QString map_path = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName;
        QDir dir(map_path);

        if(dir.exists())
        {
            QMessageBox::StandardButton reply = QMessageBox::question(
                        this,
                        "Make Sure",
                        "Same Map exist! Do You want to remake new map again?",
                        QMessageBox::Yes | QMessageBox::No);

            if(reply != QMessageBox::Yes)
            {
                ui->pushButton_StartMap->setChecked(false);
                return;
            }

            if(dir.removeRecursively())
            {
                dir.mkdir(map_path);
                QFile::Permissions permissions = QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner;
                QFile::setPermissions(map_path, permissions);
            }
            else //delete map failed
            {
                QMessageBox::warning(this,"Warning","Delate old map folder failed");
                ui->pushButton_StartMap->setChecked(false);
                return;
            }

        }
        else
        {
            dir.mkdir(map_path);
            QFile::Permissions permissions = QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner;
            QFile::setPermissions(map_path, permissions);
        }

        //startup to make map app
        QStringList tmp = map_path.split("/");
        QString pcdPath = "/";

        for(int i = 3; i< tmp.size(); i++)
            pcdPath.append(tmp.at(i) + "/");

        Utils::start_process(ros_make_map_process, "amr_ros", "om_map_liosam.launch.py savePCDDirectory:=" + pcdPath);
        Utils::start_process(ros_manual_process, "amr_ros", "om_manual.launch.py");

        ui->pushButton_StartMap->setText("Make Route App is Running");

    }
    else
    {
        //end of make map
        Utils::terminate_process(ros_manual_process);
        Utils::terminate_process(ros_make_map_process);
        ui->pushButton_StartMap->setText("Press to Start Make Map App");
    }

}

void SubWindow_MakeMap::on_pushButton_saveMap_clicked()
{
    QProcess proc;

    // 执行ros2服务调用命令
    proc.start("ros2", QStringList() << "service" << "call" << "/lio_sam/save_map" << "lio_sam/srv/SaveMap");
    // 等待执行完成，设置超时时间为5秒
    if (!proc.waitForFinished(5000))
    {
        qDebug() << "执行超时或失败";
        QMessageBox::warning(this,"Warning","Save *.pcd map Timeout");
        return;
    }

    // 获取输出信息（标准输出和标准错误）
    QString output(proc.readAllStandardOutput());
    QString errorOutput(proc.readAllStandardError());

    if (!errorOutput.isEmpty())
    {
        qDebug() << "调用服务时发生错误：" << errorOutput;
        QMessageBox::warning(this,"Warning","Save *.pcd map Failed");
    }
    else
    {
        qDebug() << "调用服务成功，返回结果：" << output;
        sendMessage("Save PCD Map Success");
    }
}

