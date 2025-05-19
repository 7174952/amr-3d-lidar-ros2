#include "utils.h"

void Utils::start_process(QProcess* rosProcess, QString package, QString fileName)
{
    if(rosProcess->state() == QProcess::NotRunning)
    {
        rosProcess->start("bash", QStringList() << "-c" << "exec setsid ros2 launch " + package + " " + fileName, QIODevice::ReadWrite);
        rosProcess->waitForStarted();
    }
}

void Utils::terminate_process(QProcess* rosProcess)
{
    if(rosProcess && (rosProcess->state() != QProcess::NotRunning))
    {
        pid_t pid = rosProcess->processId();
        // 发送 SIGTERM 信号给进程组（负的 pid 表示整个组）
        kill(-pid, SIGTERM);

        if(!rosProcess->waitForFinished(3000))
        {
            kill(-pid, SIGKILL);
            rosProcess->waitForFinished();
        }
    }
}

QString Utils::execute_shell_cmd(QString shell)
{
    QProcess proc;
    proc.start("bash", QStringList() << shell);
    if(!proc.waitForFinished(10000))
    {
        return "timeout";
    }

    QString Result = proc.readAllStandardOutput();
    return Result;
}

void Utils::start_python_script(QProcess *pyProcess, QString pythonPath, QString workDirectory, QString pyFileName)
{
    if (pyProcess->state() == QProcess::Running)
    {
        return;
    }

    QStringList arguments;
    arguments << pyFileName;

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("PYTHONUNBUFFERED", "1");
    pyProcess->setProcessEnvironment(env);
    pyProcess->setProgram(pythonPath);
    pyProcess->setArguments(arguments);
    pyProcess->setWorkingDirectory(workDirectory);

    pyProcess->start();

}

void Utils::terminate_python_script(QProcess *pyProcess)
{
    if (pyProcess->state() == QProcess::Running)
    {
        pyProcess->terminate();
        if (!pyProcess->waitForFinished(3000))
        {
            pyProcess->kill();
        }
    }
}

QVector3D Utils::transformWaypointToRobotFrame3D(const QVector3D& robot_pos, double robot_yaw, const QVector3D& waypoint_pos)
{
    // Step 1: 计算相对位置向量（在 map 坐标系下）
    double dx = waypoint_pos.x() - robot_pos.x();
    double dy = waypoint_pos.y() - robot_pos.y();
    double dz = waypoint_pos.z() - robot_pos.z();

    // Step 2: 将 dx, dy 旋转到机器人坐标系下（绕 Z 轴旋转 -yaw）
    double x_rel =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    double y_rel = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

    // z 方向不变（如果你关心 z 高度差）
    return QVector3D(x_rel, y_rel, dz);
}

double Utils::getYawFromQuaternion(double qx, double qy, double qz, double qw)
{
    // yaw (z-axis rotation)
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
}

nav_msgs::msg::Path Utils::loadWaypoints(const QString& file_path, double final_direct)
{
    nav_msgs::msg::Path waypoints_list;
    waypoints_list.header.frame_id = "map";

    QFile file(file_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        RCLCPP_ERROR(rclcpp::get_logger("robot_gui"), "Open route file: %s Failed", file_path.toStdString().c_str());
        return waypoints_list;
    }

    QTextStream line_out(&file);
    while (!line_out.atEnd())
    {
        QStringList str_tmp = line_out.readLine().split(";").at(0).split(",",Qt::SkipEmptyParts);

        if (str_tmp.size() < 7)
        {
            RCLCPP_WARN(rclcpp::get_logger("robot_gui"), "Invalid line format, skipped.");
            continue;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = str_tmp[0].toDouble();
        pose.pose.position.y = str_tmp[1].toDouble();
        pose.pose.position.z = str_tmp[2].toDouble();
        pose.pose.orientation.x = str_tmp[3].toDouble();
        pose.pose.orientation.y = str_tmp[4].toDouble();
        pose.pose.orientation.z = str_tmp[5].toDouble();
        pose.pose.orientation.w = str_tmp[6].toDouble();

        waypoints_list.poses.push_back(pose);
    }

    file.close();

    if (waypoints_list.poses.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("robot_gui"), "No waypoint to draw... Shutdown");
    }
    else
    {
        tf2::Quaternion quat_original(waypoints_list.poses.back().pose.orientation.x,
                                     waypoints_list.poses.back().pose.orientation.y,
                                     waypoints_list.poses.back().pose.orientation.z,
                                     waypoints_list.poses.back().pose.orientation.w);
        tf2::Quaternion quat_delt;
        quat_delt.setRPY(0,0, (final_direct/180.0) * M_PI);
        tf2::Quaternion quat_new = quat_original * quat_delt;
        //append last waypoint
        waypoints_list.poses.back().pose.orientation.x = quat_new.getX();
        waypoints_list.poses.back().pose.orientation.y = quat_new.getY();
        waypoints_list.poses.back().pose.orientation.z = quat_new.getZ();
        waypoints_list.poses.back().pose.orientation.w = quat_new.getW();

        RCLCPP_INFO(rclcpp::get_logger("robot_gui"), "Fetched %lu waypoints", waypoints_list.poses.size());

    }

    return waypoints_list;
}

GeoServiceTool::GeoServiceTool(rclcpp::Node::SharedPtr node, QTextEdit* textEdit_geoShowMsg)
    : node_(node), textEdit_geoShowMsg_(textEdit_geoShowMsg)
{
    datum_client_ = node_->create_client<robot_localization::srv::SetDatum>("/datum");
    fromll_client_ = node_->create_client<robot_localization::srv::FromLL>("/fromLL");
    toll_client_ = node_->create_client<robot_localization::srv::ToLL>("/toLL");

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/geo_marker", 10);
    clicked_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(&GeoServiceTool::handleClickedPoint, this, std::placeholders::_1));

    auto odom_qos = rclcpp::SensorDataQoS();
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                "/lio_sam/mapping/odometry", odom_qos,
                std::bind(&GeoServiceTool::odomCallback, this, std::placeholders::_1));
}

bool GeoServiceTool::getDatumFromFile(QString filePath, double &datum_lat, double &datum_lon, double &datum_alt, double &yaw_offset)
{
    // QString filePath = map_path + "/gnss_map_origin.txt";
    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;
    double yaw_offset_deg;
    QTextStream in(&file);
    while(!in.atEnd())
    {
        QString str = in.readLine().trimmed();
        if(str.contains("lat")) datum_lat = str.split(":").at(1).toDouble();
        if(str.contains("lon")) datum_lon = str.split(":").at(1).toDouble();
        if(str.contains("alt")) datum_alt = str.split(":").at(1).toDouble();
        if(str.contains("yaw_offset"))
        {
            yaw_offset_deg = str.split(":").at(1).toDouble();
            yaw_offset = qDegreesToRadians(yaw_offset_deg);
        }
    }
    file.close();

    if(textEdit_geoShowMsg_ != nullptr)
    {
        // 安全地发到 UI 线程更新 QTextEdit
        QString text = QString("Load Datum \nlat:%1\nlon:%2\nalt:%3\nyaw_offset(deg):%4")
                .arg(datum_lat, 0, 'f',6)
                .arg(datum_lon, 0, 'f',6)
                .arg(datum_alt, 0, 'f',2)
                .arg(yaw_offset_deg, 0, 'f',6);
        textEdit_geoShowMsg_->setPlainText(text);
    }

    return true;
}

void GeoServiceTool::setDatum(double lat, double lon, double alt)
{
    auto req = std::make_shared<robot_localization::srv::SetDatum::Request>();
    req->geo_pose.position.latitude = lat;
    req->geo_pose.position.longitude = lon;
    req->geo_pose.position.altitude = alt;

    datum_client_->async_send_request(req, [this,lat,lon,alt] (rclcpp::Client<robot_localization::srv::SetDatum>::SharedFuture result)
    {
        (void)result;
        RCLCPP_INFO(node_->get_logger(), "✅ Datum set");
        if(textEdit_geoShowMsg_ != nullptr)
        {
            // 安全地发到 UI 线程更新 QTextEdit
            QString text = QString("Set Datum Completed!\nlat:%1\nlon:%2\nalt:%3")
                    .arg(lat, 0, 'f',6)
                    .arg(lon, 0, 'f',6)
                    .arg(alt, 0, 'f',2);
            QMetaObject::invokeMethod(
              textEdit_geoShowMsg_, "setPlainText",
              Qt::QueuedConnection,
              Q_ARG(QString, text)
            );
        }
    }
    );
}

void GeoServiceTool::fromLL(double lat, double lon, double alt)
{
    auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
    req->ll_point.latitude = lat;
    req->ll_point.longitude = lon;
    req->ll_point.altitude = alt;

    fromll_client_->async_send_request(req, [this, lat, lon,alt]( rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future)
    {
        // 获取转换结果
        auto pt = future.get()->map_point;
        RCLCPP_INFO(node_->get_logger(), "📍 FromLL map: x=%.2f y=%.2f", pt.x, pt.y);
        publishMarker(pt);

        if(textEdit_geoShowMsg_ != nullptr)
        {
            QString text = QString("LL->map \nlatitude: %1\nlongitude: %2\naltitude: %3\nx: %4\ny: %5\nz: %6\n")
                     .arg(lat,  0, 'f', 3)
                     .arg(lon,  0, 'f', 3)
                     .arg(alt,  0, 'f', 2)
                     .arg(pt.x,  0, 'f', 3)
                     .arg(pt.y,  0, 'f', 3)
                     .arg(pt.z,  0, 'f', 2);
            // 安全地发到 UI 线程更新 QTextEdit
            QMetaObject::invokeMethod(
                textEdit_geoShowMsg_, "setPlainText",
                Qt::QueuedConnection,
                Q_ARG(QString, text)
            );
        }

    }
    );
}

void GeoServiceTool::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    currOdom.setX(msg->pose.pose.position.x);
    currOdom.setY(msg->pose.pose.position.y);
    currOdom.setZ(msg->pose.pose.position.z);
}

QVector3D GeoServiceTool::getCurrMapToLL()
{
    auto req = std::make_shared<robot_localization::srv::ToLL::Request>();
    req->map_point.x = currOdom.x();
    req->map_point.y = currOdom.y();
    req->map_point.z = currOdom.z();

    // 同步等待 future 返回
    auto future = toll_client_->async_send_request(req);

    // 阻塞直到结果返回（最多等待5秒）
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto geo = future.get()->ll_point;
        RCLCPP_INFO(node_->get_logger(),"🧭 ToLL: lat=%.6f lon=%.6f alt=%.2f", geo.latitude, geo.longitude, geo.altitude);

        if(textEdit_geoShowMsg_ != nullptr)
        {
            QString text = QString("map->LL \nx: %1\ny: %2\nz: %3\nlat: %4\nlon: %5\nalt: %6")
                     .arg(currOdom.x(),  0, 'f', 6)
                     .arg(currOdom.y(),  0, 'f', 6)
                     .arg(currOdom.z(),  0, 'f', 2)
                     .arg(geo.latitude,  0, 'f', 6)
                     .arg(geo.longitude, 0, 'f', 6)
                     .arg(geo.altitude,  0, 'f', 2);
            QMetaObject::invokeMethod(
                textEdit_geoShowMsg_, "setPlainText",
                Qt::QueuedConnection,
                Q_ARG(QString, text)
            );
        }

        currLL.setX(geo.latitude);
        currLL.setY(geo.longitude);
        currLL.setZ(geo.altitude);
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "🛑 ToLL service call timed out or failed.");
    }

    return currLL;
}

void GeoServiceTool::handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    auto req = std::make_shared<robot_localization::srv::ToLL::Request>();
    req->map_point = msg->point;

    toll_client_->async_send_request(req, [this, msg](rclcpp::Client<robot_localization::srv::ToLL>::SharedFuture future)
    {
        auto geo = future.get()->ll_point;
        RCLCPP_INFO(node_->get_logger(),"🧭 ToLL: lat=%.6f lon=%.6f alt=%.2f", geo.latitude, geo.longitude, geo.altitude);
        if(textEdit_geoShowMsg_ != nullptr)
        {
            QString text = QString("map->LL \nx: %1\ny: %2\nz: %3\nlat: %4\nlon: %5\nalt: %6")
                     .arg(msg->point.x,  0, 'f', 6)
                     .arg(msg->point.y,  0, 'f', 6)
                     .arg(msg->point.z,  0, 'f', 2)
                     .arg(geo.latitude,  0, 'f', 6)
                     .arg(geo.longitude, 0, 'f', 6)
                     .arg(geo.altitude,  0, 'f', 2);
            // 安全地发到 UI 线程更新 QTextEdit
            QMetaObject::invokeMethod(
                textEdit_geoShowMsg_, "setPlainText",
                Qt::QueuedConnection,
                Q_ARG(QString, text)
            );
        }
    }
    );

}

void GeoServiceTool::publishMarker(const geometry_msgs::msg::Point &point)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "geo_marker";
    marker.id = 0;
    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.8;
    marker.color.b = 0.1;
    marker_pub_->publish(marker);
}

double GeoServiceTool::getGnssMoveOrientalDeg(GnssPostion aheadPointPos, GnssPostion refPointPos)
{
    // 地球半径（米）
    const double R = 6378137.0;
    double dLat = qDegreesToRadians(aheadPointPos.lat - refPointPos.lat);
    double dLon = qDegreesToRadians(aheadPointPos.lon - refPointPos.lon);
    double latRefRad = qDegreesToRadians(refPointPos.lat);

    double x_east = R * dLon * qCos(latRefRad);   // 东方向
    double y_north = R * dLat;                    // 北方向

    double angle_rad = qAtan2(y_north, x_east);  // atan2(Δ北, Δ东)
    double angle_deg = qRadiansToDegrees(angle_rad);
    angle_deg =  fmod(angle_deg + 360.0, 360.0);       // 归一化到 [0, 360)
    return angle_deg;

}
