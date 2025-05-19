#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/distances.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <QString>
#include <QRegExp>
#include <QStringList>
#include <QDebug>

class PclObstDetector : public rclcpp::Node
{
public:
    PclObstDetector();

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obst_points;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_obst_points_num;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloudPoints;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdRaw;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_naviRules;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_guide;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_voice_control;

    struct Navi_Rules
    {
        bool rule_enable;
        double limit_max_vel;
        double width_tolerance;
    };
    Navi_Rules navi_rules;

    struct Guide_Info
    {
        bool guide_en;
        double speed_rate;
    };
    Guide_Info guide_info;

    bool voice_control_en;
    QString voice_control_cmd;

    double OBST_HIGHT_MIN_Z;
    double OBST_HIGHT_MAX_Z;
    double linear_velocity = 0;
    double turning_radius = 0;
    double offset_dist = 0.1;
    bool is_near_goal = false;
    int32_t obstacle_points_num = 0;
    int32_t obstacle_lim = 0;

    double robot_width_size;
    double robot_width_tolerance;
    double obst_speed_rate = 0;

    double calculateSpeedReduction();
    void cloudPoints_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
    void cmdRaw_Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void naviRules_Callback(const std_msgs::msg::String::SharedPtr msg);
    void guideControl_Callback(const std_msgs::msg::String::SharedPtr msg);
    void voiceControl_Callback(const std_msgs::msg::String::SharedPtr msg);
};

PclObstDetector::PclObstDetector() : Node("pcl_obst_detector")
{
    this->declare_parameter("OBST_HIGH_MIN_Z", -0.1);
    this->declare_parameter("OBST_HIGH_MAX_Z", 1.0);
    this->declare_parameter("obstacle_lim", 10);
    this->declare_parameter("robot_width_size", 0.52);

    this->get_parameter("OBST_HIGH_MIN_Z", OBST_HIGHT_MIN_Z);
    this->get_parameter("OBST_HIGH_MAX_Z", OBST_HIGHT_MAX_Z);
    this->get_parameter("obstacle_lim", obstacle_lim);
    this->get_parameter("robot_width_size", robot_width_size);

    pub_obst_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_points", 10);
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_obst_points_num = this->create_publisher<std_msgs::msg::Int32>("/obstacle_points_num", 10);

    sub_cloudPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&PclObstDetector::cloudPoints_Callback, this, std::placeholders::_1)
    );

    sub_cmdRaw = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_raw", 10,
        std::bind(&PclObstDetector::cmdRaw_Callback, this, std::placeholders::_1)
    );

    sub_naviRules = this->create_subscription<std_msgs::msg::String>(
        "/navi_rules", 10,
        std::bind(&PclObstDetector::naviRules_Callback, this, std::placeholders::_1)
    );
    sub_guide = this->create_subscription<std_msgs::msg::String>(
        "/guide_control", 10,
        std::bind(&PclObstDetector::guideControl_Callback, this, std::placeholders::_1)
    );
    sub_voice_control = this->create_subscription<std_msgs::msg::String>(
                "/voice_control", 10,
                std::bind(&PclObstDetector::voiceControl_Callback, this, std::placeholders::_1)
            );
    navi_rules = {false, 0.0, 0.0};
    guide_info = {false, 0.0};
}

double PclObstDetector::calculateSpeedReduction()
{
    if (obstacle_points_num > obstacle_lim)
    {
        obst_speed_rate -= 0.25;
        obst_speed_rate = std::max(obst_speed_rate, 0.0);
    }
    else
    {
        obst_speed_rate = 1.0;
    }
    return obst_speed_rate;
}

void PclObstDetector::cloudPoints_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_input);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_input);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_input);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(OBST_HIGHT_MIN_Z, OBST_HIGHT_MAX_Z);
    pass.setInputCloud(cloud_input);
    pass.filter(*cloud_input);

    double obstal_width = robot_width_size + (navi_rules.rule_enable ? navi_rules.width_tolerance : 0.0) * 2;

    const double ratio = 2.0;
    double max_filter_dist = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (turning_radius != 0)
    {
        double center_angle = (linear_velocity * ratio) / turning_radius;
        pcl::CropBox<pcl::PointXYZ> boxFilter;

        if (turning_radius > 0)
        {
            boxFilter.setMin(Eigen::Vector4f(0, -obstal_width / 2, OBST_HIGHT_MIN_Z, 1.0));
            max_filter_dist = turning_radius * std::sin(center_angle);
            boxFilter.setMax(Eigen::Vector4f(max_filter_dist, turning_radius, OBST_HIGHT_MAX_Z, 1.0));
        }
        else
        {
            boxFilter.setMin(Eigen::Vector4f(0, turning_radius, OBST_HIGHT_MIN_Z, 1.0));
            max_filter_dist = turning_radius * std::sin(center_angle);
            boxFilter.setMax(Eigen::Vector4f(max_filter_dist, obstal_width / 2, OBST_HIGHT_MAX_Z, 1.0));
        }

        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*cloud_input);

        double inner_radius = std::abs(turning_radius) - obstal_width / 2;
        double outer_radius = std::abs(turning_radius) + obstal_width / 2;

        for (const auto& point : cloud_input->points)
        {
            double point_distance = std::sqrt(point.x * point.x + (point.y - turning_radius) * (point.y - turning_radius));
            if (point_distance <= outer_radius && point_distance >= inner_radius)
            {
                filtered_cloud->points.push_back(point);
            }
        }
    }
    else
    {
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(0.0, -obstal_width / 2, OBST_HIGHT_MIN_Z, 1.0));
        max_filter_dist = linear_velocity * ratio;
        boxFilter.setMax(Eigen::Vector4f(max_filter_dist, obstal_width / 2, OBST_HIGHT_MAX_Z, 1.0));
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*filtered_cloud);
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    obstacle_points_num = filtered_cloud->points.size();

    std_msgs::msg::Int32 points_num;
    points_num.data = obstacle_points_num;
    pub_obst_points_num->publish(points_num);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud, output);
    output.header.frame_id = "livox_frame";
    pub_obst_points->publish(output);
}

void PclObstDetector::voiceControl_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    QStringList voice_control_msg = QString::fromStdString(msg->data).split(";",Qt::SkipEmptyParts);
    voice_control_en = voice_control_msg.at(0).split(":").at(1).toUInt();
    voice_control_cmd = voice_control_msg.at(1);

}

void PclObstDetector::guideControl_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    QStringList guide_msg = QString::fromStdString(msg->data).split(";",Qt::SkipEmptyParts);
    guide_info.guide_en = guide_msg.at(0).split(":").at(1).toUInt();
    guide_info.speed_rate = guide_msg.at(1).split(":").at(1).toDouble();

}

void PclObstDetector::cmdRaw_Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto cmd_vel = geometry_msgs::msg::Twist();

    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.angular.z = msg->angular.z;

    if(guide_info.guide_en)
    {
        cmd_vel.linear.x = cmd_vel.linear.x * guide_info.speed_rate;
        cmd_vel.angular.z = cmd_vel.angular.z * guide_info.speed_rate;
    }

    //check navi rules
    if(navi_rules.rule_enable)
    {
        cmd_vel.linear.x = std::min(navi_rules.limit_max_vel, cmd_vel.linear.x);
    }

    if(voice_control_en && voice_control_cmd == "stop")
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }

    if (cmd_vel.angular.z != 0)
    {
        turning_radius = cmd_vel.linear.x / cmd_vel.angular.z;
        linear_velocity = cmd_vel.linear.x;
    }
    else
    {
        turning_radius = 0;
        linear_velocity = cmd_vel.linear.x;
    }

    double reduction_ratio = calculateSpeedReduction();
    cmd_vel.linear.x *= reduction_ratio;
    cmd_vel.angular.z *= reduction_ratio;

    pub_cmd_vel->publish(cmd_vel);
}

void PclObstDetector::naviRules_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    QStringList rules_msg = QString::fromStdString(msg->data).split(";");

    for(const QString &rule : rules_msg)
    {
        QStringList item = rule.split(":");
        if(item.at(0) == "rule_en")
        {
            navi_rules.rule_enable = (item.at(1) == "true") ? true : false;
        }
        if(item.at(0) == "max_vel")
        {
            navi_rules.limit_max_vel = item.at(1).toDouble();
        }
        if(item.at(0) == "width_tole")
        {
            navi_rules.width_tolerance = item.at(1).toDouble();
        }
        //add other rules if exist
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclObstDetector>());
    rclcpp::shutdown();
    return 0;
}
