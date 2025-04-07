#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <QProcess>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <QtGui/QVector3D>
#include <QDir>
#include <cmath>
#include <QDebug>

#include "global_dataset.h"
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>


namespace Ui {
    class Utils;
}

class Utils
{
public:
    static void start_process(QProcess *, QString, QString);
    static void terminate_process(QProcess *);
    static QString execute_shell_cmd(QString);
    static void start_python_script(QProcess *, QString);
    static void terminate_python_script(QProcess *);

    static QVector3D transformWaypointToRobotFrame3D(const QVector3D&, double , const QVector3D& );
    static double getYawFromQuaternion(double qx, double qy, double qz, double qw);
    static nav_msgs::msg::Path loadWaypoints(const QString& file_path, double final_direct);

    template<typename T1, typename T2>
    static double distance(const T1 &pt1, const T2 &pt2)
    {
        return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
    }


private:
    Utils() = delete;
    ~Utils() = delete;

};

#endif // UTILS_H
