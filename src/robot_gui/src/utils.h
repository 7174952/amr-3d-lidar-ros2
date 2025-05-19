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
#include <QTextEdit>
#include <QMetaObject>
#include <QVector3D>
#include <QtMath>

#include "global_dataset.h"
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/set_datum.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <robot_localization/srv/to_ll.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace Ui {
    class Utils;
    class GeoServiceTool;
}

class Utils
{
public:
    static void start_process(QProcess *, QString, QString);
    static void terminate_process(QProcess *);
    static QString execute_shell_cmd(QString);
    static void start_python_script(QProcess *, QString, QString, QString);
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

class GeoServiceTool
{
public:
    struct GnssPostion
    {
        double lat;
        double lon;
        double alt;
        double yaw_offset;
    };

public:
    GeoServiceTool(rclcpp::Node::SharedPtr node, QTextEdit* textEdit_geoShowMsg);

    void setDatum(double lat, double lon, double alt);
    void fromLL(double lat, double lon, double alt);
    void handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    QVector3D getCurrMapToLL();
    bool getDatumFromFile(QString filePath, double &datum_lat, double &datum_lon, double &datum_alt, double &yaw_offset);
    double getGnssMoveOrientalDeg(GnssPostion aheadPointPos, GnssPostion refPointPos);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr datum_client_;
    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr fromll_client_;
    rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr toll_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    QTextEdit* textEdit_geoShowMsg_;
    QVector3D currOdom;
    QVector3D currLL;

    void publishMarker(const geometry_msgs::msg::Point &point);
};

#endif // UTILS_H
