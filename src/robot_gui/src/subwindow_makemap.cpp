#include "subwindow_makemap.h"

SubWindow_MakeMap::SubWindow_MakeMap(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_MakeMap)
{
    ui->setupUi(this);

    ros_make_map_process = new QProcess(this);;
    ros_manual_process = new QProcess(this);
    gnss_visual_process = new QProcess(this);

    is_init_datum = false;
    geoTool = new GeoServiceTool(node_, ui->textEdit_mapStatus);
    gpsConvThreshold = Global_DataSet::instance().gnssNtrip()["ConvThreshold"].toDouble();
}

SubWindow_MakeMap::~SubWindow_MakeMap()
{
    Utils::terminate_process(ros_manual_process);
    Utils::terminate_process(ros_make_map_process);
    Utils::terminate_python_script(gnss_visual_process);
    delete ui;
}

void SubWindow_MakeMap::updateMapName(const QString& newMapName)
{
    m_mapName = newMapName;
}

void SubWindow_MakeMap::onNewFixReceived(int status, double conv, double lat, double lon, double alt)
{
    if(!is_init_datum && conv < gpsConvThreshold && ui->pushButton_StartMap->isChecked())
    {
        is_init_datum = true;
        geoTool->setDatum(origin_lat, origin_lon, origin_alt);
    }
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
        QString srcPath = map_path + "/gnss_map_origin.txt";
        QString dstPath = Global_DataSet::instance().sysPath()["MapPath"] + "/gnss_map_origin.txt";

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
            //backup gnss_map_origin.txt
            if(QFile::exists(srcPath))
            {
                QFile::copy(srcPath, dstPath);
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
        //recover gnss_map_origin.txt
        if(QFile::exists(dstPath))
        {
            QFile::copy(dstPath, srcPath);
            QFile::remove(dstPath);
        }

        is_init_datum = false;
        //startup to make map app
        QStringList tmp = map_path.split("/");
        QString pcdPath = "/";

        for(int i = 3; i< tmp.size(); i++)
            pcdPath.append(tmp.at(i) + "/");

        bool gnss_enable = (Global_DataSet::instance().sensorEnable("GnssEn") && ui->checkBox_mapWithGnss->isChecked());
        if(gnss_enable)
        {
            QString filePath = map_path + "/gnss_map_origin.txt";
            QFile file(filePath);
            if(file.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QTextStream in(&file);
                while(!in.atEnd())
                {
                    QString str = in.readLine().trimmed();
                    if(str.contains("lat")) origin_lat = str.split(":").at(1).toDouble();
                    if(str.contains("lon")) origin_lon = str.split(":").at(1).toDouble();
                    if(str.contains("alt")) origin_alt = str.split(":").at(1).toDouble();
                    if(str.contains("yaw_offset")) yaw_offset = qDegreesToRadians(str.split(":").at(1).toDouble());
                }
            }
            file.close();
        }

        QString strCmd = QString("om_map_liosam.launch.py")
                + " savePCDDirectory:=" + pcdPath
                + " enable_gnss:=" + (gnss_enable ? "true" : "false");
        if(gnss_enable)
        {
            strCmd += " latitude:=" + QString::number(origin_lat, 'f', 6)
                    + " longitude:=" + QString::number(origin_lon,'f',6)
                    + " altitude:=" + QString::number(origin_alt,'f',2)
                    + " yaw_offset:=" + QString::number(yaw_offset,'f',6)
                    + " gpsCovThreshold:=" + QString::number(gpsConvThreshold,'f', 3);
        }

        Utils::start_process(ros_make_map_process, "amr_ros", strCmd);
        Utils::start_process(ros_manual_process, "amr_ros", "om_manual.launch.py");
        ui->pushButton_StartMap->setText("Make Route App is Running");

        if(Global_DataSet::instance().sensorEnable("GnssEn"))
        {
            QString filePath = Global_DataSet::instance().sysPath()["ScriptPath"] + "/gps_visualizer.py";
            QString pythonPath = "/bin/python3";
            QString workDirectory = Global_DataSet::instance().sysPath()["ScriptPath"];

            Utils::start_python_script(gnss_visual_process, pythonPath, workDirectory,filePath);
        }
    }
    else
    {
        //end of make map
        Utils::terminate_process(ros_manual_process);
        Utils::terminate_process(ros_make_map_process);
        Utils::terminate_python_script(gnss_visual_process);
        ui->pushButton_StartMap->setText("Press to Start Make Map App");
    }

}

void SubWindow_MakeMap::on_pushButton_saveMap_clicked()
{
    QString map_path = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName;
    QString srcPath = map_path + "/gnss_map_origin.txt";
    QString dstPath = Global_DataSet::instance().sysPath()["MapPath"] + "/gnss_map_origin.txt";
    if(QFile::exists(srcPath))
    {
        QFile::copy(srcPath, dstPath);
    }

    QProcess proc;

    // 执行ros2服务调用命令
    proc.start("ros2", QStringList() << "service" << "call" << "/lio_sam/save_map" << "lio_sam/srv/SaveMap");
    // 等待执行完成，设置超时时间为30秒
    if (!proc.waitForFinished(30000))
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
        ui->textEdit_mapStatus->append("Save Map Success");
    }

    if(QFile::exists(dstPath))
    {
        QFile::copy(dstPath, srcPath);
        QFile::remove(dstPath);
    }

}


