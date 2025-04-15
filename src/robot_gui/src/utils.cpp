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
