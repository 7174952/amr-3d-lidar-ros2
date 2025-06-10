#ifndef SUBWINDOW_MAKEROUTE_H
#define SUBWINDOW_MAKEROUTE_H

#include "ui_subwindow_makeroute.h"

#include <QDialog>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QListView>
#include <QStringListModel>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QRegExp>

//ROS2 Files
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>


#include "utils.h"
#include "global_dataset.h"


namespace Ui {
class SubWindow_MakeRoute;
}

class SubWindow_MakeRoute : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_MakeRoute(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SubWindow_MakeRoute();

protected:
    void closeEvent(QCloseEvent *event) override
    {
        emit subWindowClosed();
        QDialog::closeEvent(event);
    }

private:
    //define waypoints list
    struct Robot_Pose
    {
        double pos_x;
        double pos_y;
        double pos_z;
        double ori_x;
        double ori_y;
        double ori_z;
        double ori_w;
    };
    Robot_Pose robot_cur_pose;
    Robot_Pose init_pose;

    struct Waypoint_Info
    {
        QString name;
        Robot_Pose pose;
    };
    Waypoint_Info init_location_info;

    QList<QString> route_records;
    double odom_distance_now;
    double route_odom_start;
    Robot_Pose pose_route_from;
    Robot_Pose Pose_route_to;

    //define rule
    struct Robot_Position
    {
        double pos_x;
        double pos_y;
        double pos_z;
    };
    struct Rule
    {
        Robot_Position pos_in;  // begin
        Robot_Position pos_out; // end
        //rules
        double max_vel;
        double width_tolerance;
        void clear()
        {
            pos_in = {0,0,0};
            pos_out = {0,0,0};
            max_vel = 0.0;
            width_tolerance = 0.0;
        }
    };
    Rule rule;
    QList<Rule> rules_list;


signals:
    void subWindowClosed();
    void odomReceived(QVector<double> odom_pose);

public slots:
    void updateMapName(const QString& newMapName);
    void onNavsatStartupCompleted();

private slots:
    void on_pushButton_StartRoute_toggled(bool checked);

    void on_pushButton_recordRoute_toggled(bool checked);

    void on_pushButton_exchLocation_clicked();

    void Odometry_CallBack(const nav_msgs::msg::Odometry& odom);

    void on_pushButton_withRule_toggled(bool checked);

    void on_comboBox_initLocation_currentTextChanged(const QString &arg1);

private:
    void upload_routeList();

private:
    rclcpp::Node::SharedPtr node_;
    QProcess* ros_make_route_process;
    QProcess* ros_manual_process;

    QString m_mapName;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    QVector<double> last_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    int idx_;

    QMap<QString, Robot_Pose> init_location_list;

private:
    Ui::SubWindow_MakeRoute *ui;
    QStandardItemModel *model;
    GeoServiceTool *geoTool;
    double origin_lat;
    double origin_lon;
    double origin_alt;
    double yaw_offset;
    bool is_init_datum;
    double gpsConvThreshold;

};

#endif // SUBWINDOW_MAKEROUTE_H
