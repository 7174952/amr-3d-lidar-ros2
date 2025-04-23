#include "subwindow_guiderobot.h"
#include "ui_subwindow_guiderobot.h"

SubWindow_GuideRobot::SubWindow_GuideRobot(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_GuideRobot)
{
    ui->setupUi(this);

    initConfig();

    guide_robot_process = new QProcess(this);
    chatbot_process = new QProcess(this);

    detect_process = new QProcess(this);
#if 1 //debug_ryu
    connect(chatbot_process, &QProcess::readyReadStandardOutput, this, &SubWindow_GuideRobot::handleOutput);
    connect(chatbot_process, &QProcess::readyReadStandardError, this, &SubWindow_GuideRobot::handleError);
#endif

    //init location and pose
    robot_cur_pose = {0,0,0,0,0,0,1};
    init_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    last_point.resize(7);
    last_point.fill(0.0);

    odom_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&SubWindow_GuideRobot::Odometry_CallBack, this, std::placeholders::_1));
    reached_goal_sub = node_->create_subscription<std_msgs::msg::Bool>(
                "/reached_goal", 1, std::bind(&SubWindow_GuideRobot::reachedGoal_CallBack, this, std::placeholders::_1));
    connect(this, &SubWindow_GuideRobot::odomReceived, this, &SubWindow_GuideRobot::onSearchCurrLocation);
    connect(this, &SubWindow_GuideRobot::odomReceived, this, &SubWindow_GuideRobot::onSetNaviRules);
    person_detect_sub = node_->create_subscription<std_msgs::msg::String>(
                "/person_detect", 10, std::bind(&SubWindow_GuideRobot::personDetect_CallBack, this, std::placeholders::_1));
    obstacle_num_sub = node_->create_subscription<std_msgs::msg::Int32>(
                "/obstacle_points_num", 10, std::bind(&SubWindow_GuideRobot::obstacle_CallBack, this, std::placeholders::_1));
    guide_pub = node_->create_publisher<std_msgs::msg::String>("/guide_control", 10);

    rules_pub = node_->create_publisher<std_msgs::msg::String>("/navi_rules", 1);
    marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("/waypoint_marker", 10);
    final_direct = 0;
    reached_goal = false;

    //load and public route path
    waypoints_pub = node_->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);
    reset_client = node_->create_client<std_srvs::srv::Empty>("/reset_path");

    //chatbot
    chatbot_state_sub = node_->create_subscription<std_msgs::msg::String>(
                "/chatbot_state", 1, std::bind(&SubWindow_GuideRobot::chatbot_state_CallBack, this, std::placeholders::_1));
    chatbot_request = node_->create_client<std_srvs::srv::SetBool>("/chatbot_wakeup");
    QString filePath = Global_DataSet::instance().sysPath()["ScriptPath"] + "/chatbot/chatbot.py";
    QString pythonPath = Global_DataSet::instance().sysPath()["PyPathVoice"] + "/bin/python";
    QString workDirectory = Global_DataSet::instance().sysPath()["ScriptPath"] + "/chatbot";
    Utils::start_python_script(chatbot_process, pythonPath, workDirectory, filePath);


    ui->comboBox_maxVel->setCurrentText("0.6");
    ui->listView_targets_list->setSpacing(8);
    ui->listView_targets_list->setItemDelegate(new ButtonCheckDelegate());
    ui->listView_targets_list->setEditTriggers(QAbstractItemView::NoEditTriggers);

    baseModel = new QStandardItemModel(this);
    proxyModel = new FilterProxy;
    proxyModel->setSourceModel(baseModel);
    ui->listView_targets_list->setModel(proxyModel);
    connect(ui->listView_targets_list, &QListView::clicked, this, &SubWindow_GuideRobot::onItemClicked);

    //camera person detect
    person_info.curr_num = 0;
    person_info.last_num = 0;
    person_info.cnt = 0;
    person_info.guide_en = false;
    person_info.near_distance = std::numeric_limits<double>::infinity();
    person_info.speed_rate = 0.0;

    //init sound manager
    audioManager = new AudioManager(this);
    system_path = Global_DataSet::instance().sysPath();
    obstacle_points_num = 0;
    audioManager->setObstacleAudio(system_path["ResourcePath"] + "/obstacle_alert_ja.mp3");
    audioManager->setGuideAudio(system_path["ResourcePath"] + "/guide_follow_up_ja.mp3");
    audioManager->setLoopIntervalForGuide(2000);
    audioManager->setLoopIntervalForObstacle(2000);
    guide_annouse = false;
    connect(audioManager, &AudioManager::greetFinished, this, &SubWindow_GuideRobot::onGreetVoiceFinished);
    is_greet = false;

}

SubWindow_GuideRobot::~SubWindow_GuideRobot()
{
    saveConfig();
    Utils::terminate_python_script(chatbot_process);
    Utils::terminate_process(detect_process);
    Utils::terminate_process(guide_robot_process);

    delete ui;
}

void SubWindow_GuideRobot::saveConfig()
{
    QSettings settings_navi("mikuni", "GuideRobot/Navi");

    settings_navi.setValue("GuideFunctionEn", ui->checkBox_cameraGuide->isChecked());
}

void SubWindow_GuideRobot::initConfig()
{
    QSettings settings_navi("mikuni", "GuideRobot/Navi");
    bool guide_function_en = settings_navi.value("GuideFunctionEn", "false").toBool();
    ui->checkBox_cameraGuide->setChecked(guide_function_en);
    Global_DataSet::instance().setGuideFunctionEn(guide_function_en);

}

void SubWindow_GuideRobot::onGreetVoiceFinished()
{
    if(is_greet)
    {
        is_greet = false;
        wakeupChatbot(true); //wakeup mode
    }

}

void SubWindow_GuideRobot::onItemClicked(const QModelIndex &proxyIndex)
{
    QModelIndex sourceIndex = proxyModel->mapToSource(proxyIndex);
    QString text = sourceIndex.data(Qt::DisplayRole).toString();
    bool checked = (sourceIndex.data(Qt::CheckStateRole).toInt() == Qt::Checked);

    final_direct = checked ?  180:0.0;
    navi_target = text;
}

void SubWindow_GuideRobot::onSearchCurrLocation()
{
    QString last_location = current_location;
    //search current location
    if(ui->pushButton_NaviGo->isChecked()) //guide running
    {
        double dist_start = std::sqrt(  std::pow(location_list[navi_start].x - robot_cur_pose.pos_x, 2)
                              + std::pow(location_list[navi_start].y- robot_cur_pose.pos_y, 2)
                              + std::pow(location_list[navi_start].z - robot_cur_pose.pos_z, 2));

        double dist_target = std::sqrt(  std::pow(location_list[navi_target].x - robot_cur_pose.pos_x, 2)
                              + std::pow(location_list[navi_target].y- robot_cur_pose.pos_y, 2)
                              + std::pow(location_list[navi_target].z - robot_cur_pose.pos_z, 2));
        if(dist_start < 1.0)
        {
            current_location = navi_start;
        }
        else if(dist_target < 1.0)
        {
            //change current location
            current_location = navi_target;
            navi_start = current_location;
        }
        else
        {
            current_location = navi_start + "->" + navi_target;
        }
    }
    else //guide stop
    {
        current_location = "";
        for(auto it = location_list.cbegin(); it != location_list.cend(); it++)
        {
            double dist = std::sqrt(  std::pow(it.value().x - robot_cur_pose.pos_x, 2)
                                    + std::pow(it.value().y- robot_cur_pose.pos_y, 2)
                                    + std::pow(it.value().z - robot_cur_pose.pos_z, 2));
            if(dist < 1.0)
            {
                current_location = it.key();
                navi_start = current_location;
                break;
            }
        }
        if(current_location.isEmpty())
        {
            current_location = "unknown";
        }
    }
    ui->lineEdit_currentLocation->setText(current_location);

    //show targets
    if((last_location != current_location) &&  location_list.contains(current_location))
    {
        for(const Route_Info &route: route_list)
        {
            if(route.start == current_location)
            {
                QStandardItem *item = new QStandardItem(route.target);
                item->setCheckable(true); // 可勾选
                item->setCheckState(Qt::Unchecked);
                item->setEditable(false);
                baseModel->appendRow(item);
            }
        }
    }
    else if(current_location.contains("unknown") || current_location.contains("->"))
    {
        baseModel->clear();
    }

}

void SubWindow_GuideRobot::onSetNaviRules()
{
    if(rules_route.rules_list.isEmpty()) // no rules
        return;
    if(rule_state.cnt >= rules_route.rules_list.size())
        return;

    if(ui->pushButton_NaviGo->isChecked())
    {
        double robot_yaw = Utils::getYawFromQuaternion(robot_cur_pose.ori_x, robot_cur_pose.ori_y, robot_cur_pose.ori_z, robot_cur_pose.ori_w);
        QVector3D rule_waypoint, waypoint_relative;
        QVector3D robot_pos(robot_cur_pose.pos_x, robot_cur_pose.pos_y, robot_cur_pose.pos_z);

        if(rule_state.status == "out")
        {
            rule_waypoint.setX(rules_route.rules_list.at(rule_state.cnt).pos_in.x);
            rule_waypoint.setY(rules_route.rules_list.at(rule_state.cnt).pos_in.y);
            rule_waypoint.setZ(rules_route.rules_list.at(rule_state.cnt).pos_in.z);
            //Calculate waypoint relative positon base on robot
            waypoint_relative = Utils::transformWaypointToRobotFrame3D(robot_pos, robot_yaw, rule_waypoint);
            if(waypoint_relative.x() < 0)
            {
                //publish and enable rules
                QString msg_tmp = QString("rule_en:true;")
                        + "max_vel:" + QString::number(rules_route.rules_list.at(rule_state.cnt).max_vel,'f',2) + ";"
                        + "width_tole:" + QString::number(rules_route.rules_list.at(rule_state.cnt).width_tolerance/100,'f',2);
                std_msgs::msg::String msg;
                msg.data = msg_tmp.toStdString();
                rules_pub->publish(msg);
                rule_state.status = "in";
            }
        }
        else // "in"
        {
            rule_waypoint.setX(rules_route.rules_list.at(rule_state.cnt).pos_out.x);
            rule_waypoint.setY(rules_route.rules_list.at(rule_state.cnt).pos_out.y);
            rule_waypoint.setZ(rules_route.rules_list.at(rule_state.cnt).pos_out.z);
            //Calculate waypoint relative positon base on robot
            waypoint_relative = Utils::transformWaypointToRobotFrame3D(robot_pos, robot_yaw, rule_waypoint);
            if(waypoint_relative.x() < 0)
            {
                //publish and disable rules
                QString msg_tmp = QString("rule_en:false;")
                        + "max_vel:" + QString::number(rules_route.rules_list.at(rule_state.cnt).max_vel,'f',2) + ";"
                        + "width_tole:" + QString::number(rules_route.rules_list.at(rule_state.cnt).width_tolerance/100,'f',2);
                std_msgs::msg::String msg;
                msg.data = msg_tmp.toStdString();
                rules_pub->publish(msg);
                rule_state.status = "out";
                rule_state.cnt++;
            }
        }

    }
}

void SubWindow_GuideRobot::handleOutput()
{
    QString output = chatbot_process->readAllStandardOutput();
    qDebug() << output;
}

void SubWindow_GuideRobot::handleError()
{
    QString error = chatbot_process->readAllStandardError();
    qDebug() << error;
}

void SubWindow_GuideRobot::onGuideCameraStateChanged(const int arg)
{
    bool state = (arg == 2) ? true : false;
    Global_DataSet::instance().setSensorEnable("GuideCameraEn", state);
    //startup guide camera
    if(state && ui->pushButton_startRobot->isChecked())
    {
        QString filePath = Global_DataSet::instance().sysPath()["ScriptPath"] + "/person_detect.py";
        QString pythonPath = Global_DataSet::instance().sysPath()["PyPathCamera"] + "/bin/python";
        QString workDirectory = Global_DataSet::instance().sysPath()["ScriptPath"];

        Utils::start_python_script(detect_process,pythonPath, workDirectory, filePath);
    }
    //stop camera
    if(!state)
    {
        Utils::terminate_python_script(detect_process);
    }
    ui->checkBox_cameraGuide->setEnabled(state);
}

void SubWindow_GuideRobot::chatbot_state_CallBack(const std_msgs::msg::String& msg)
{
    QString str = QString::fromStdString(msg.data);
    QString state = str.left(str.indexOf(';')).split(":").at(1);
    QString qa_msg = str.mid(str.indexOf(';') + 1);

    if(state == "sleep")
    {

    }
    else if(state == "ready")
    {

    }
    else if(state == "wakeup")
    {
        audioManager->stopGreet();
    }
    else if(state == "question")
    {
        ui->textEdit->append("Q:" + qa_msg);
    }
    else if(state == "answer")
    {
        ui->textEdit->append("A:" + qa_msg);
    }
    else if(state == "speech")
    {
        audioManager->setGreetAudio(system_path["ScriptPath"] + "/chatbot/assets/speech.mp3");
        audioManager->playGreet();
    }
    else
    {
        //do nothing
    }

    chatbot_state = state;
    emit sendMessage("chatbot state:" + state);
}

void SubWindow_GuideRobot::obstacle_CallBack(const std_msgs::msg::Int32& msg)
{
    //obstacle appeared
    if((msg.data > 10) && (obstacle_points_num < 10))
    {
        audioManager->startObstacle();
    }
    //obstacle disappeared
    if((msg.data < 10) && (obstacle_points_num > 10))
    {
        audioManager->stopObstacle();
    }
    obstacle_points_num = msg.data;
}

void SubWindow_GuideRobot::Odometry_CallBack(const nav_msgs::msg::Odometry& odom)
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
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_pub->publish(marker);
    }

    emit this->odomReceived();

}

void SubWindow_GuideRobot::personDetect_CallBack(const std_msgs::msg::String& msg)
{
    if(!ui->checkBox_cameraGuide->isChecked() || !ui->pushButton_NaviGo->isChecked())
        return;
    //check distance between robot and first waypoint and the last waypoint
    Robot_Position robot_position = {robot_cur_pose.pos_x, robot_cur_pose.pos_y, robot_cur_pose.pos_z};
    if(  Utils::distance(robot_position, waypoints_msg.poses.front().pose.position) < 4.0 //Not guide in 2.0 meters when start
      || Utils::distance(robot_position, waypoints_msg.poses.back().pose.position) < 4.0) //Not guide in 2.0 meters at last
    {
        //publish :disable
        std_msgs::msg::String guide_msg;
        guide_msg.data = "guide_en:0;speed_rate:0.0;";
        guide_pub->publish(guide_msg);
        if(guide_annouse)
        {
            guide_annouse = false;
            audioManager->stopGuide();
        }
        return;
    }

    QStringList info = QString::fromStdString(msg.data).trimmed().split(QRegExp(";"), Qt::SkipEmptyParts);
    person_info.curr_num = info.at(0).split(":").at(1).toUInt();
    if(    (person_info.curr_num > 0 && person_info.last_num == 0)
        || (person_info.curr_num == 0 && person_info.last_num > 0))
    {
        person_info.cnt = 0;
    }
    else
    {
        person_info.cnt++;
    }
    person_info.last_num = person_info.curr_num;

    //filter
    if(person_info.cnt < person_info.DETECT_MAX_TIMES)
        return;

    if(person_info.curr_num == 0)
    {
        //publish: enable, stop
        std_msgs::msg::String guide_msg;
        guide_msg.data = "guide_en:1;speed_rate:0.0;";
        guide_pub->publish(guide_msg);

        if(!guide_annouse)
        {
            guide_annouse = true;
            audioManager->startGuide(); //tell guest to follow up
        }
        return;
    }
    //get nearest person distance
    info.removeFirst(); //delete total num
    person_info.near_distance = std::numeric_limits<double>::infinity();
    for(const QString& str : info)
    {
        double distance = str.split(":").at(1).toDouble();
        person_info.near_distance = std::min(distance, person_info.near_distance);
    }

    //calculate speed rate
    if(person_info.near_distance > person_info.FAR_STOP)
    {
        person_info.speed_rate -= 0.1;
        person_info.speed_rate = std::max(person_info.speed_rate, 0.0);
        if(!guide_annouse)
        {
            guide_annouse = true;
            audioManager->startGuide(); //tell guest to follow up
        }
    }
    else if(person_info.near_distance < person_info.FAR_START)
    {
        person_info.speed_rate += 0.1;
        person_info.speed_rate = std::min(person_info.speed_rate, 1.0);
        if(guide_annouse)
        {
            guide_annouse = false;
            audioManager->stopGuide(); //tell guest to follow up
        }
    }
    //publish: enable, run by speed_rate
    std_msgs::msg::String guide_msg;
    guide_msg.data = "guide_en:1;speed_rate:" + QString::number(person_info.speed_rate,'f',2).toStdString() + ";";
    guide_pub->publish(guide_msg);

}

void SubWindow_GuideRobot::reachedGoal_CallBack(const std_msgs::msg::Bool& msg)
{
    reached_goal = msg.data;
    if (reached_goal)
    {
        ui->pushButton_NaviGo->setChecked(false);
    }
}

void SubWindow_GuideRobot::updateMapName(const QString& newMapName)
{
    m_mapName = newMapName;

    upload_LocationList();
    upload_RouteList();
}

void SubWindow_GuideRobot::upload_LocationList()
{
    QString filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/location_info.txt";
    QFile file(filePath);
    if(!file.exists())
    {
        QMessageBox::warning(this,"Warning","locations info file is not exist!");
    }
    else
    {
        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warning","Open locations info file Error");
        }
        else
        {
            QTextStream in(&file);
            while (!in.atEnd())
            {
                QStringList strList = in.readLine().split(QRegExp(":|,|;"),Qt::SkipEmptyParts);
                if(strList.size() < 4)
                    continue;

                Robot_Position position;
                QString name = strList.at(0);
                position.x = strList.at(1).toDouble();
                position.y = strList.at(2).toDouble();
                position.z = strList.at(3).toDouble();
                location_list[name] = position;
            }
            file.close();
        }
    }
}

void SubWindow_GuideRobot::upload_RouteList()
{
    QString filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/base_route_info.txt";
    QFile file(filePath);
    if(!file.exists())
    {
        QMessageBox::warning(this,"Warning","base_route info file is not exist!");
    }
    else
    {
        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this,"Warning","Open base_route info file Error");
        }
        else
        {
            QTextStream in(&file);
            Route_Info route_info;
            route_info.clear();

            while (!in.atEnd())
            {
                QString line = in.readLine();

                if(line.contains("->")) //route
                {
                    QStringList strList = line.split(QRegExp(":|;"),Qt::SkipEmptyParts);
                    if(strList.size() < 2) continue;
                    else
                    {
                        if(route_info.start.size() > 0) //add to route list
                        {
                            route_list[route_info.start + "->" + route_info.target] = route_info;
                            route_info.clear();
                        }
                        route_info.start = strList.at(0).split("->").at(0);
                        route_info.target = strList.at(0).split("->").at(1);
                        route_info.distance = strList.at(1).toDouble();

                    }

                }
                else if(line.contains("~")) //rules
                {
                    if(route_info.start.isEmpty()) continue;
                    else
                    {
                        QStringList strList = line.split(QRegExp("\\(|\\)|~|:=|,|;"),Qt::SkipEmptyParts);
                        if(strList.size() < 8) continue;
                        {
                            //add to rule list
                            Rule rule;
                            rule.pos_in.x = strList.at(0).toDouble();
                            rule.pos_in.y = strList.at(1).toDouble();
                            rule.pos_in.z = strList.at(2).toDouble();
                            rule.pos_out.x = strList.at(3).toDouble();
                            rule.pos_out.y = strList.at(4).toDouble();
                            rule.pos_out.z = strList.at(5).toDouble();
                            rule.max_vel = strList.at(6).toDouble();
                            rule.width_tolerance = strList.at(7).toDouble();
                            route_info.rules_list.append(rule);
                        }

                    }
                }
            }
            //add final route
            if(route_info.start.size() > 0)
            {
                route_list[route_info.start + "->" + route_info.target] = route_info;
            }
            file.close();

        }
    }
}

void SubWindow_GuideRobot::on_pushButton_startRobot_toggled(bool checked)
{
    if(checked)
    {
        //startup to make route app
        QString map_path = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/GlobalMap.pcd";

        QString strCmd = QString("om_navi_guide_robot.launch.py")
                + " v_max:=" + ui->comboBox_maxVel->currentText()
                + " OBST_HIGHT_MIN_Z:=" + Global_DataSet::instance().robotSize()["ObstRangeMin"]
                + " OBST_HIGHT_MAX_Z:=" + Global_DataSet::instance().robotSize()["ObstRangeMax"]
                + " robot_width_size:=" + Global_DataSet::instance().robotSize()["RobotWidth"]

                + " globalmap_path:=" + map_path
                + " init_px:=" + QString::number(init_pose.pos_x,'f')
                + " init_py:=" + QString::number(init_pose.pos_y,'f')
                + " init_pz:=" + QString::number(init_pose.pos_z,'f')
                + " init_ox:=" + QString::number(init_pose.ori_x,'f')
                + " init_oy:=" + QString::number(init_pose.ori_y,'f')
                + " init_oz:=" + QString::number(init_pose.ori_z,'f')
                + " init_ow:=" + QString::number(init_pose.ori_w,'f');

        Utils::start_process(guide_robot_process, "amr_ros", strCmd);

        //startup guide camera
        QString filePath = Global_DataSet::instance().sysPath()["ScriptPath"] + "/person_detect.py";
        QString pythonPath = Global_DataSet::instance().sysPath()["PyPathCamera"] + "/bin/python";
        QString workDirectory = Global_DataSet::instance().sysPath()["ScriptPath"];
        if(Global_DataSet::instance().sensorEnable("GuideCameraEn"))
        {
            Utils::start_python_script(detect_process, pythonPath, workDirectory,filePath);
        }

        ui->pushButton_startRobot->setText("Robot App is Running");
        audioManager->setGreetAudio(system_path["ResourcePath"] + "/greet_startup_navi_ja.mp3");
        audioManager->playGreet();
        is_greet = true;
    }
    else
    {
        wakeupChatbot(false);
        //terminate application
        // Utils::terminate_python_script(chatbot_process);
        Utils::terminate_python_script(detect_process);
        Utils::terminate_process(guide_robot_process);
        ui->pushButton_startRobot->setText("Press to Start Robot App");
        baseModel->clear();
        audioManager->setGreetAudio(system_path["ResourcePath"] + "/greet_finish_navi_ja.mp3");
        audioManager->playGreet();

    }

}

void SubWindow_GuideRobot::wakeupChatbot(bool req)
{
    // 等待 service 可用
    if (!chatbot_request->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available.");
        return;
    }

    // 发起请求
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = req;  // 请求数据

    // 调用服务
    auto future = chatbot_request->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Set chatbot ready");
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Set chatbot failed.");
        QMessageBox::warning(this,"Warning","Set chatbot failed.");
    }

}

void SubWindow_GuideRobot::on_pushButton_NaviGo_toggled(bool checked)
{
    if(checked)
    {
        //Load route path
        QString fileName = navi_start + "_" + navi_target + ".txt";
        if(navi_start == "unknown")
        {
            QMessageBox::warning(this,"Warning","Current location is unknown, Please init Robot postion on rviz first!");
            ui->pushButton_NaviGo->setChecked(false);
            return;
        }
        QString route_path = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName
                            + "/base_route/" + fileName;
        if(!QFile::exists(route_path))
        {
            QMessageBox::warning(this,"Warning","Route file " + fileName + " not exist!");
            ui->pushButton_NaviGo->setChecked(false);
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        waypoints_msg = Utils::loadWaypoints(route_path, final_direct);

        // 等待服务可用
        if (!reset_client->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_WARN(node_->get_logger(), "Service reset_path not available.");
            QMessageBox::warning(this,"Warning","Service reset_path not available.");
            ui->pushButton_NaviGo->setChecked(false);
            return;
        }

        waypoints_pub->publish(waypoints_msg);

        // 调用服务
        auto future = reset_client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node_->get_logger(), "Reset path succeeded.");
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Reset path failed.");
            QMessageBox::warning(this,"Warning","Reset path failed.");
            ui->pushButton_NaviGo->setChecked(false);
            return;
        }

        //Init route rules
        rules_route = route_list[navi_start + "->" + navi_target];
        rule_state.cnt = 0;
        rule_state.status = "out";

        wakeupChatbot(false); //sleep mode
        ui->pushButton_NaviGo->setText("Moving to " + navi_target);
        audioManager->setGreetAudio(system_path["ResourcePath"] + "/goto_next_target_ja.mp3");
        audioManager->playGreet();

    }
    else
    {
        ui->pushButton_NaviGo->setText("GO");
        ui->pushButton_NaviGo->blockSignals(true);
        ui->pushButton_NaviGo->setChecked(false);
        ui->pushButton_NaviGo->blockSignals(false);

        audioManager->setGreetAudio(system_path["ResourcePath"] + "/essay/" + navi_target + "_ja.mp3");
        audioManager->playGreet();
        wakeupChatbot(true);  // 唤醒模式
    }
}

void SubWindow_GuideRobot::on_checkBox_cameraGuide_stateChanged(int arg1)
{
    bool state = (arg1 == 2) ? true : false;
    Global_DataSet::instance().setGuideFunctionEn(state);
}

