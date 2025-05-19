#include "subwindow_makeroute.h"

SubWindow_MakeRoute::SubWindow_MakeRoute(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_MakeRoute)
{
    ui->setupUi(this);

    ros_make_route_process = new QProcess(this);
    ros_manual_process = new QProcess(this);

    sub_odom = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SubWindow_MakeRoute::Odometry_CallBack, this, std::placeholders::_1));

    marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("/waypoint_marker", 10);

    robot_cur_pose = {0,0,0,0,0,0,1};
    init_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    last_point.resize(7);
    last_point.fill(0.0);
    odom_distance_now = 0.0;

    geoTool = new GeoServiceTool(node_, ui->textEdit_makeRouteMsg);
    gpsConvThreshold = Global_DataSet::instance().gnssNtrip()["ConvThreshold"].toDouble();

}

SubWindow_MakeRoute::~SubWindow_MakeRoute()
{
    Utils::terminate_process(ros_manual_process);
    Utils::terminate_process(ros_make_route_process);

    delete ui;
}

void SubWindow_MakeRoute::Odometry_CallBack(const nav_msgs::msg::Odometry& odom)
{
    //update current position and orient
    robot_cur_pose.pos_x = odom.pose.pose.position.x;
    robot_cur_pose.pos_y = odom.pose.pose.position.y;
    robot_cur_pose.pos_z = odom.pose.pose.position.z;
    robot_cur_pose.ori_x = odom.pose.pose.orientation.x;
    robot_cur_pose.ori_y = odom.pose.pose.orientation.y;
    robot_cur_pose.ori_z = odom.pose.pose.orientation.z;
    robot_cur_pose.ori_w = odom.pose.pose.orientation.w;

    double distance = std::sqrt(  std::pow(last_point[0] - robot_cur_pose.pos_x, 2)
                                + std::pow(last_point[1] - robot_cur_pose.pos_y, 2)
                                + std::pow(last_point[2] - robot_cur_pose.pos_z, 2));

    const double waypoint_interval = 0.25;

    if (distance >= waypoint_interval)
    {

        last_point[0] = robot_cur_pose.pos_x;
        last_point[1] = robot_cur_pose.pos_y;
        last_point[2] = robot_cur_pose.pos_z;
        last_point[3] = robot_cur_pose.ori_x;
        last_point[4] = robot_cur_pose.ori_y;
        last_point[5] = robot_cur_pose.ori_z;
        last_point[6] = robot_cur_pose.ori_w;
        odom_distance_now += distance;

        //save current pose to route file
        if(ui->pushButton_recordRoute->isChecked())
        {
            QString str;
            for(int i = 0; i < last_point.size(); i++)
            {
                str.append(QString::number(last_point.at(i),'f'));
                if(i < last_point.size() - 1)
                    str.append(",");
                else
                    str.append(";");
            }
            //set to buffer
            route_records.push_back(str);
        }

        // 生成 Marker 并发布
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        auto now = node_->get_clock()->now();
        marker.header.stamp.sec = static_cast<uint32_t>(now.nanoseconds() / 1000000000);
        marker.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
        marker.id = idx_++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = robot_cur_pose.pos_x;
        marker.pose.position.y = robot_cur_pose.pos_y;
        marker.pose.position.z = robot_cur_pose.pos_z;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_pub->publish(marker);
    }
}

void SubWindow_MakeRoute::upload_routeList()
{
    QStringList route_items;
    route_items.append("[From]->[To]:[Distance(m)]");

    QString filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/base_route_info.txt";
    QFile routeInfoFile(filePath);
    //read info from file
    if(routeInfoFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&routeInfoFile);

        while (!in.atEnd())
        {
            QString line = in.readLine();
            if(line.contains("->"))
                route_items.append(line.trimmed().split(";",Qt::SkipEmptyParts).at(0));
        }
        routeInfoFile.close();
    }
    model = new QStandardItemModel();
    //change backgroud color
    for (int i = 0; i < route_items.size(); i++)
    {
        QStandardItem *item = new QStandardItem(route_items[i]);

        if(i == 0)
            item->setBackground(QColor("#ffffff"));
        else if (i % 2 == 0)
            item->setBackground(QColor("#f0f0f0"));
        else
            item->setBackground(QColor("#d0eaff"));

        model->appendRow(item);
    }
    ui->listView_allRoutes->setModel(model);
}

void SubWindow_MakeRoute::updateMapName(const QString& newMapName)
{
    m_mapName = newMapName;

    //show all route details
    upload_routeList();

}

void SubWindow_MakeRoute::onNavsatStartupCompleted()
{
    geoTool->setDatum(origin_lat, origin_lon, origin_alt);
}

void SubWindow_MakeRoute::on_pushButton_StartRoute_toggled(bool checked)
{
    if(checked)
    {   
        //init to calculate distance
        last_point[0] = robot_cur_pose.pos_x;
        last_point[1] = robot_cur_pose.pos_y;
        last_point[2] = robot_cur_pose.pos_z;
        last_point[3] = robot_cur_pose.ori_x;
        last_point[4] = robot_cur_pose.ori_y;
        last_point[5] = robot_cur_pose.ori_z;
        last_point[6] = robot_cur_pose.ori_w;

        //startup to make route app
        QString map_path = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName;

        bool gnss_enable = (Global_DataSet::instance().sensorEnable("GnssEn") && ui->checkBox_naviWithGnss->isChecked());
        QString strCmd = QString("om_navi_make_route.launch.py")
                + " globalmap_path:=" + map_path + "/GlobalMap.pcd"
                + " init_px:=" + QString::number(init_pose.pos_x,'f')
                + " init_py:=" + QString::number(init_pose.pos_y,'f')
                + " init_pz:=" + QString::number(init_pose.pos_z,'f')
                + " init_ox:=" + QString::number(init_pose.ori_x,'f')
                + " init_oy:=" + QString::number(init_pose.ori_y,'f')
                + " init_oz:=" + QString::number(init_pose.ori_z,'f')
                + " init_ow:=" + QString::number(init_pose.ori_w,'f')
                + " enable_gnss:=" + (gnss_enable ? "true" : "false");

        if(gnss_enable)
        {
            QString filePath = map_path + "/gnss_map_origin.txt";
            gpsConvThreshold = Global_DataSet::instance().gnssNtrip()["ConvThreshold"].toDouble();

            if(!geoTool->getDatumFromFile(filePath, origin_lat, origin_lon, origin_alt, yaw_offset))
            {
                QMessageBox::warning(this,"Warning","Load Datum from file Failed! check gnss_map_origin.txt file");
                return;
            }
            strCmd += " latitude:=" + QString::number(origin_lat, 'f', 6)
                    + " longitude:=" + QString::number(origin_lon,'f',6)
                    + " altitude:=" + QString::number(origin_alt,'f',2)
                    + " yaw_offset:=" + QString::number(yaw_offset,'f',6)
                    + " gps_cov_threshold:=" + QString::number(gpsConvThreshold,'f',3);
        }

        Utils::start_process(ros_make_route_process, "amr_ros", strCmd);
        Utils::start_process(ros_manual_process, "amr_ros", "om_manual.launch.py");

        ui->pushButton_StartRoute->setText("Make Route App is Running");

        //Create folder /base_route
        QString folderPath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/base_route";
        QDir dir(folderPath);
        if(!dir.exists())
        {
            dir.mkdir(folderPath);
            QFile::Permissions permissions = QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner;
            QFile::setPermissions(folderPath, permissions);
        }
    }
    else
    {
        //terminate application
        Utils::terminate_process(ros_manual_process);
        Utils::terminate_process(ros_make_route_process);
        ui->pushButton_StartRoute->setText("Press to Start Make Route App");
    }

}


void SubWindow_MakeRoute::on_pushButton_recordRoute_toggled(bool checked)
{
    if(checked) //start
    {
        //Check if same location
        if(ui->comboBox_from->currentText() == ui->comboBox_routeTo->currentText())
        {
            QMessageBox::warning(this,"Warnig","Cannot make route from [" + ui->comboBox_from->currentText()
                                        + "] to [" + ui->comboBox_routeTo->currentText() + "]");
            ui->pushButton_recordRoute->setChecked(false);
            return;
        }
        //check if base route exist
        QString file_name = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/base_route/base_" + ui->comboBox_from->currentText() + "_"
                            + ui->comboBox_routeTo->currentText() + ".txt";
        if(QFile::exists(file_name))
        {
            QMessageBox::StandardButton reply = QMessageBox::question(
                        this,
                        "Make Sure",
                        "Same Route exist! Do You want to make new route again?",
                        QMessageBox::Yes | QMessageBox::No);
            if(reply == QMessageBox::No)
            {
                ui->pushButton_recordRoute->setChecked(false);
                return;
            }
        }

        ui->pushButton_recordRoute->setText("Recording");
        //init route data
        route_records.clear();
        route_odom_start = odom_distance_now;
        pose_route_from = robot_cur_pose;

    }
    else //stop
    {
        //save route
        QString filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/base_route/"
                            + ui->comboBox_from->currentText() + "_" + ui->comboBox_routeTo->currentText() + ".txt";
        QFile file(filePath);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
        {
            QTextStream out(&file);
            for(const QString& rec_str : route_records)
                out << rec_str << "\n";
            file.close();
        }
        else
        {
            qDebug() << "Save route file error" << file.errorString();
            QMessageBox::warning(this,"Warnig","Save route file error");
        }

        //update base_route_info.txt
        filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/base_route_info.txt";
        QString tempFilePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/temp_file.txt";

        QString routeName = ui->comboBox_from->currentText() + "->" + ui->comboBox_routeTo->currentText();

        QFile routeInfoFile(filePath);
        QFile tempFile(tempFilePath);

        if(!routeInfoFile.exists())
        {
            //create file
            if(routeInfoFile.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                QTextStream out(&routeInfoFile);
                //save new route info
                out << routeName << ":" << QString::number(odom_distance_now - route_odom_start,'f',2) << ";" << "\n";
                for(const Rule tmp_rule : rules_list)
                {
                    out << "("
                        << QString::number(tmp_rule.pos_in.pos_x,'f') << ","
                        << QString::number(tmp_rule.pos_in.pos_y,'f') << ","
                        << QString::number(tmp_rule.pos_in.pos_z,'f') << ")"
                        << "~"
                        << "("
                        << QString::number(tmp_rule.pos_out.pos_x,'f') << ","
                        << QString::number(tmp_rule.pos_out.pos_y,'f') << ","
                        << QString::number(tmp_rule.pos_out.pos_z,'f') << ")"
                        << ":="
                        << QString::number(tmp_rule.max_vel) << ","
                        << QString::number(tmp_rule.width_tolerance) << ";"
                        << "\n";
                }
                routeInfoFile.close();
                //show in listView
                QStandardItem *item = new QStandardItem(routeName);
                model->appendRow(item);
            }
        }
        else
        {
            if (routeInfoFile.open(QIODevice::ReadOnly | QIODevice::Text) &&
                tempFile.open(QIODevice::WriteOnly | QIODevice::Text))
            {

                QTextStream in(&routeInfoFile);
                QTextStream out(&tempFile);
                bool is_searched = false;
                bool is_route_exist = false;

                while (!in.atEnd())
                {
                    QString line = in.readLine();
                    if(line.contains(routeName)) //skip old route
                    {
                        is_searched = true;
                        is_route_exist = true;
                        continue;
                    }
                    else
                    {
                        if(!is_searched) //skip old rules
                        {
                            out << line << "\n";
                            continue;
                        }
                        if(line.contains("->"))
                        {
                            out << line << "\n";
                            is_searched = false;
                        }
                    }
                }

                //save new route info
                double route_distance = odom_distance_now - route_odom_start;
                out << routeName << ":" << QString::number(route_distance,'f',2) << ";\n";
                for(const Rule tmp_rule : rules_list)
                {
                    out << "("
                        << QString::number(tmp_rule.pos_in.pos_x,'f') << ","
                        << QString::number(tmp_rule.pos_in.pos_y,'f') << ","
                        << QString::number(tmp_rule.pos_in.pos_z,'f') << ")"
                        << "~"
                        << "("
                        << QString::number(tmp_rule.pos_out.pos_x,'f') << ","
                        << QString::number(tmp_rule.pos_out.pos_y,'f') << ","
                        << QString::number(tmp_rule.pos_out.pos_z,'f') << ")"
                        << ":="
                        << QString::number(tmp_rule.max_vel) << ","
                        << QString::number(tmp_rule.width_tolerance) << ";"
                        << "\n";
                }
                routeInfoFile.close();
                tempFile.close();

                // 替换原文件
                routeInfoFile.remove();
                tempFile.rename(filePath);

                if(!is_route_exist)
                {
                    QStandardItem *item = new QStandardItem(routeName + ":" + QString::number(route_distance,'f',2));
                    model->appendRow(item);

                }
            }
        }

        //update location_info.txt
        filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/location_info.txt";
        tempFilePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/temp_file.txt";
        Pose_route_to = robot_cur_pose;

        QFile locationInfoFile(filePath);
        QFile tempLocationInfoFile(tempFilePath);

        if(!locationInfoFile.exists())
        {
            //create file
            if(locationInfoFile.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                QTextStream out(&locationInfoFile);
                //save location info
                out << ui->comboBox_from->currentText() << ":"
                    << QString::number(pose_route_from.pos_x,'f') << ","
                    << QString::number(pose_route_from.pos_y,'f') << ","
                    << QString::number(pose_route_from.pos_z,'f') << ";\n";

                out << ui->comboBox_routeTo->currentText() << ":"
                    << QString::number(Pose_route_to.pos_x,'f') << ","
                    << QString::number(Pose_route_to.pos_y,'f') << ","
                    << QString::number(Pose_route_to.pos_z,'f') << ";\n";

                locationInfoFile.close();
            }
        }
        else
        {
            if (locationInfoFile.open(QIODevice::ReadOnly | QIODevice::Text) &&
                tempLocationInfoFile.open(QIODevice::WriteOnly | QIODevice::Text))
            {

                QTextStream in(&locationInfoFile);
                QTextStream out(&tempLocationInfoFile);
                bool is_location_from_exist = false;
                bool is_location_to_exist = false;
                while (!in.atEnd())
                {
                    QString line = in.readLine();
                    out << line << "\n";

                    if (ui->comboBox_from->currentText() == line.trimmed().split(":").at(0))
                    {
                        is_location_from_exist = true;
                    }
                    else if (ui->comboBox_routeTo->currentText() == line.trimmed().split(":").at(0))
                    {
                        is_location_to_exist = true;
                    }
                }
                //add new
                if(!is_location_from_exist)
                {
                    out << ui->comboBox_from->currentText() << ":"
                        << pose_route_from.pos_x << ","
                        << pose_route_from.pos_y << ","
                        << pose_route_from.pos_z << ";\n";
                }
                if(!is_location_to_exist)
                {
                    out << ui->comboBox_routeTo->currentText() << ":"
                        << Pose_route_to.pos_x << ","
                        << Pose_route_to.pos_y << ","
                        << Pose_route_to.pos_z << ";\n";
                }

                locationInfoFile.close();
                tempLocationInfoFile.close();

                // 替换原文件
                locationInfoFile.remove();
                tempLocationInfoFile.rename(filePath);
            }
        }

        rules_list.clear();
        ui->pushButton_recordRoute->setText("Press to Record");

    }

}


void SubWindow_MakeRoute::on_pushButton_exchLocation_clicked()
{
    QString str = ui->comboBox_from->currentText();
    ui->comboBox_from->setCurrentText(ui->comboBox_routeTo->currentText());
    ui->comboBox_routeTo->setCurrentText(str);
}


void SubWindow_MakeRoute::on_pushButton_withRule_toggled(bool checked)
{
    if(checked)
    {
        rule.clear();
        rule.pos_in.pos_x = robot_cur_pose.pos_x;
        rule.pos_in.pos_y = robot_cur_pose.pos_y;
        rule.pos_in.pos_z = robot_cur_pose.pos_z;
        rule.max_vel = ui->comboBox_maxVel->currentText().toDouble();
        rule.width_tolerance = ui->comboBox_widthTolerance->currentText().toDouble();
        ui->pushButton_withRule->setText("Rules Enabled");
    }
    else
    {
        rule.pos_out.pos_x = robot_cur_pose.pos_x;
        rule.pos_out.pos_y = robot_cur_pose.pos_y;
        rule.pos_out.pos_z = robot_cur_pose.pos_z;
        rules_list.append(rule);
        ui->pushButton_withRule->setText("Press to Enable Rules");
    }
}

