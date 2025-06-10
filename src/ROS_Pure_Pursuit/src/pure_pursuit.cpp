#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/utils.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <kdl/frames.hpp>
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using std::string;

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

    void cmd_generator(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void goal_orientation_control(const geometry_msgs::msg::Pose& goal_pose, bool *orient_reached);
    std::vector<std::string> split(const std::string& str, char delimiter);
    bool reset_path_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void waypoints_listener(const nav_msgs::msg::Path::SharedPtr new_path);
    // 将全局坐标下的位姿转换到 base_link 坐标系下
    KDL::Frame trans2base(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Transform& tf);

    template<typename T1, typename T2>
    double distance(const T1 &pt1, const T2 &pt2)
    {
        // return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
        return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

private:
    double wheel_base_;
    double lookahead_distance_, position_tolerance_;
    double orientation_tolerance_;
    double v_max_, v_, w_max_;
    double delta_, delta_vel_, acc_, dec_, jerk_, delta_max_;
    int idx_memory;
    unsigned idx_;
    bool goal_reached_, path_loaded_;
    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::Twist cmd_vel_;
    ackermann_msgs::msg::AckermannDriveStamped cmd_acker_;
    visualization_msgs::msg::Marker lookahead_marker_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_acker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_turn_round_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_path_srv;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::msg::TransformStamped lookahead_;
    std::string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_;
    bool reset_path_en;
    bool orient_reached;
    bool init_orient_reached;
    bool set_wait;
    double wait_time;
};

PurePursuit::PurePursuit() : Node("pure_pursuit"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),tf_broadcaster_(this)
{
    // 声明并获取参数
    this->declare_parameter<double>("wheelbase", 1.0);
    this->declare_parameter<double>("lookahead_distance", 1.0);
    this->declare_parameter<double>("max_linear_velocity", 0.2);
    this->declare_parameter<double>("max_rotational_velocity", 1.0);
    this->declare_parameter<double>("position_tolerance", 0.1);
    this->declare_parameter<double>("orientation_tolerance", 0.1);
    this->declare_parameter<double>("steering_angle_velocity", 0.1);
    this->declare_parameter<double>("acceleration", 0.1);
    this->declare_parameter<double>("deceleration", 0.1);
    this->declare_parameter<double>("jerk", 0.1);
    this->declare_parameter<double>("steering_angle_limit", 0.5);
    this->declare_parameter<string>("map_frame_id", "map");
    this->declare_parameter<string>("robot_frame_id", "base_link");
    this->declare_parameter<string>("lookahead_frame_id", "lookahead");
    this->declare_parameter<string>("ackermann_frame_id", "ackermann");

    this->get_parameter("wheelbase", wheel_base_);
    this->get_parameter("lookahead_distance", lookahead_distance_);
    this->get_parameter("max_linear_velocity", v_max_);
    this->get_parameter("max_rotational_velocity", w_max_);
    this->get_parameter("position_tolerance", position_tolerance_);
    this->get_parameter("orientation_tolerance", orientation_tolerance_);
    this->get_parameter("steering_angle_velocity", delta_vel_);
    this->get_parameter("acceleration", acc_);
    this->get_parameter("deceleration", dec_);
    this->get_parameter("jerk", jerk_);
    this->get_parameter("steering_angle_limit", delta_max_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("robot_frame_id", robot_frame_id_);
    this->get_parameter("lookahead_frame_id", lookahead_frame_id_);
    this->get_parameter("ackermann_frame_id", acker_frame_id_);

    RCLCPP_INFO(this->get_logger(), "Get parameters completed");


    lookahead_.header.frame_id = robot_frame_id_;
    lookahead_.child_frame_id = lookahead_frame_id_;

    cmd_acker_.header.frame_id = acker_frame_id_;
    cmd_acker_.drive.steering_angle_velocity = delta_vel_;
    cmd_acker_.drive.acceleration = acc_;
    cmd_acker_.drive.jerk = jerk_;

    v_ = 0;
    idx_memory = 0;
    path_loaded_ = false;

    // 订阅话题
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>("/waypoints", 10, std::bind(&PurePursuit::waypoints_listener, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PurePursuit::cmd_generator, this, std::placeholders::_1));

    // 发布话题
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_raw", 10);
    pub_acker_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("cmd_acker", 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead", 10);
    pub_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>("/reached_goal", 10);
    pub_start_turn_round_ = this->create_publisher<std_msgs::msg::Bool>("/start_turn_round",10);

    // 创建服务
    reset_path_srv = this->create_service<std_srvs::srv::Empty>("reset_path", std::bind(&PurePursuit::reset_path_callback, this, std::placeholders::_1, std::placeholders::_2));

    reset_path_en = false;
    orient_reached = false;
    init_orient_reached = false;
    set_wait = false;
}

// 根据当前位姿和路径点生成控制指令
void PurePursuit::cmd_generator(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    odom_ = *odom_msg;

    if (path_loaded_)
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            if (init_orient_reached == false)
            {
                idx_ = 0;
                std_msgs::msg::Bool start_turn_round;
                start_turn_round.data = true;
                pub_start_turn_round_->publish(start_turn_round);

            }
            else
            {
                tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero);
                // 根据当前位姿、路径信息和前视距离确定跟踪的路径点
                for (idx_ = idx_memory; idx_ < path_.poses.size(); idx_++)
                {
                    if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > lookahead_distance_)
                    {
                        KDL::Frame pose_offset = trans2base(path_.poses[idx_].pose, tf.transform);

                        lookahead_.transform.translation.x = std::isnan(pose_offset.p.x()) ? 0.0 : pose_offset.p.x();
                        lookahead_.transform.translation.y = std::isnan(pose_offset.p.y()) ? 0.0 : pose_offset.p.y();
                        lookahead_.transform.translation.z = std::isnan(pose_offset.p.z()) ? 0.0 : pose_offset.p.z();

                        double x_t,y_t,z_t, w_t;
                        pose_offset.M.GetQuaternion(x_t, y_t, z_t, w_t);
                        lookahead_.transform.rotation.x = std::isnan(x_t) ? 0.0 : x_t;
                        lookahead_.transform.rotation.y = std::isnan(y_t) ? 0.0 : y_t;
                        lookahead_.transform.rotation.z = std::isnan(z_t) ? 0.0 : z_t;
                        lookahead_.transform.rotation.w = std::isnan(w_t) ? 0.0 : w_t;

                        idx_memory = idx_;
                        break;
                    }
                }
            }
            // 接近目标（最后一个路径点）
            if (!path_.poses.empty() && idx_ >= path_.poses.size())
            {
                KDL::Frame goal_offset = trans2base(path_.poses.back().pose, tf.transform);

                // 判断是否到达目标位置或达到目标方向
                if ((!goal_reached_ && (std::abs(goal_offset.p.x()) <= position_tolerance_)) ||
                    (goal_reached_ && !orient_reached))
                {
                    goal_reached_ = true;

                    std_msgs::msg::Bool start_turn_round;
                    start_turn_round.data = true;
                    pub_start_turn_round_->publish(start_turn_round);

                    geometry_msgs::msg::Pose goal_pose = path_.poses.back().pose;
                    goal_orientation_control(goal_pose, &orient_reached);
                    if (orient_reached)
                    {
                        // 到达目标后重置路径
                        path_ = nav_msgs::msg::Path();
                        std_msgs::msg::Bool is_reached_goal;
                        is_reached_goal.data = true;
                        pub_goal_reached_->publish(is_reached_goal);

                        std_msgs::msg::Bool start_turn_round;
                        start_turn_round.data = false;
                        pub_start_turn_round_->publish(start_turn_round);

                    }
                    else
                    {
                        return;
                    }
                }
                else
                {
                    // 若未满足位置公差，则延伸前视距离至目标外
                    double roll, pitch, yaw;
                    goal_offset.M.GetRPY(roll, pitch, yaw);
                    double k_end = tan(yaw);
                    double l_end = goal_offset.p.y() - k_end * goal_offset.p.x();
                    double a = 1 + k_end * k_end;
                    double b = 2 * l_end;
                    double c = l_end * l_end - lookahead_distance_ * lookahead_distance_;
                    double D = sqrt(b * b - 4 * a * c);
                    double x_ld = (-b + copysign(D, v_)) / (2 * a);
                    double y_ld = k_end * x_ld + l_end;

                    lookahead_.transform.translation.x = std::isnan(x_ld) ? 0.0 : x_ld;
                    lookahead_.transform.translation.y = std::isnan(y_ld) ? 0.0 : y_ld;
                    lookahead_.transform.translation.z = std::isnan(goal_offset.p.z()) ? 0.0 : goal_offset.p.z();

                    double x_t, y_t, z_t, w_t;
                    goal_offset.M.GetQuaternion(x_t, y_t,z_t, w_t);
                    lookahead_.transform.rotation.x = std::isnan(x_t) ? 0.0 : x_t;
                    lookahead_.transform.rotation.y = std::isnan(y_t) ? 0.0 : y_t;
                    lookahead_.transform.rotation.z = std::isnan(z_t) ? 0.0 : z_t;
                    lookahead_.transform.rotation.w = std::isnan(w_t) ? 0.0 : w_t;
                }
            }
            // 路径跟踪
            if (!goal_reached_)
            {
                // 起步前转向对齐
                if ((idx_ == 0) && (init_orient_reached == false))
                {
                    geometry_msgs::msg::Pose goal_pose = path_.poses.front().pose;
                    goal_orientation_control(goal_pose, &init_orient_reached);
                    // 等待车辆稳定
                    if (init_orient_reached && set_wait == false)
                    {
                        init_orient_reached = false;
                        set_wait = true;
                        wait_time = this->get_clock()->now().seconds();
                    }

                    if (set_wait)
                    {
                        if (this->get_clock()->now().seconds() - wait_time > 3.0)
                        {
                            init_orient_reached = true;

                            std_msgs::msg::Bool start_turn_round;
                            start_turn_round.data = false;
                            pub_start_turn_round_->publish(start_turn_round);

                            set_wait = false;
                            v_ = 0;
                        }
                        else
                        {
                            init_orient_reached = false;
                        }
                    }
                }
                else
                {
                    if(distance(odom_.pose.pose.position, path_.poses.back().pose.position) > 1.5) //speed up
                    {
                        v_ += acc_;
                        v_ = std::min(v_, v_max_);
                    }
                    else // robot距离目的地1.5m时减速
                    {
                        v_ -= dec_;
                        v_ = std::max(0.3, v_);
                    }

                    double lateral_offset = lookahead_.transform.translation.y;
                    cmd_vel_.angular.z = std::min(2 * v_ / lookahead_distance_ * lookahead_distance_ * lateral_offset, w_max_);
                    // 计算 Ackermann 转向角
                    cmd_acker_.drive.steering_angle = std::min(atan2(2 * lateral_offset * wheel_base_, lookahead_distance_ * lookahead_distance_), delta_max_);
                    // 设定线速度
                    cmd_vel_.linear.x = v_;
                    cmd_acker_.drive.speed = v_;
                    cmd_acker_.header.stamp = this->get_clock()->now();
                }
            }
            // 到达目标后停车
            else
            {
                if (reset_path_en)
                {
                    // 重启
                    idx_ = 0;
                    idx_memory = 0;
                    goal_reached_ = false;
                    reset_path_en = false;
                    orient_reached = false;
                    init_orient_reached = false;
                    std_msgs::msg::Bool is_reached_goal;
                    is_reached_goal.data = false;
                    pub_goal_reached_->publish(is_reached_goal);

                }
                else
                {
                    cmd_vel_.linear.x = 0.00;
                    cmd_vel_.angular.z = 0.00;
                    cmd_acker_.header.stamp = this->get_clock()->now();
                    cmd_acker_.drive.steering_angle = 0.00;
                    cmd_acker_.drive.speed = 0.00;
                }
            }

            // 发布前视目标变换
            lookahead_.header.stamp = this->get_clock()->now();
            tf_broadcaster_.sendTransform(lookahead_);
            // 发布速度指令
            cmd_vel_.angular.z = std::isnan(cmd_vel_.angular.z) ? 0.0 : cmd_vel_.angular.z;
            cmd_vel_.angular.z = std::copysign(std::min(std::fabs(cmd_vel_.angular.z), w_max_), cmd_vel_.angular.z);
            pub_vel_->publish(cmd_vel_);
            // 发布 Ackermann 控制指令
            pub_acker_->publish(cmd_acker_);
            // 发布前视点可视化 marker
            lookahead_marker_.header.frame_id = "lookahead";
            lookahead_marker_.header.stamp = this->get_clock()->now();
            lookahead_marker_.type = visualization_msgs::msg::Marker::SPHERE;
            lookahead_marker_.action = visualization_msgs::msg::Marker::ADD;
            lookahead_marker_.scale.x = 0.1;
            lookahead_marker_.scale.y = 0.1;
            lookahead_marker_.scale.z = 0.1;
            lookahead_marker_.pose.orientation.x = 0.0;
            lookahead_marker_.pose.orientation.y = 0.0;
            lookahead_marker_.pose.orientation.z = 0.0;
            lookahead_marker_.pose.orientation.w = 1.0;
            lookahead_marker_.color.a = 1.0;
            if (!goal_reached_)
            {
                lookahead_marker_.id = idx_;
                lookahead_marker_.pose.position.x = path_.poses[idx_].pose.position.x;
                lookahead_marker_.pose.position.y = path_.poses[idx_].pose.position.y;
                lookahead_marker_.pose.position.z = path_.poses[idx_].pose.position.z;
                lookahead_marker_.color.r = 0.0;
                lookahead_marker_.color.g = 1.0;
                lookahead_marker_.color.b = 0.0;
                pub_marker_->publish(lookahead_marker_);
            }
            else
            {
                lookahead_marker_.id = idx_memory;
                idx_memory += 1;
                lookahead_marker_.pose.position.x = std::isnan(tf.transform.translation.x) ? 0.0 : tf.transform.translation.x;
                lookahead_marker_.pose.position.y = std::isnan(tf.transform.translation.y) ? 0.0 : tf.transform.translation.y;
                lookahead_marker_.pose.position.z = std::isnan(tf.transform.translation.z) ? 0.0 : tf.transform.translation.z;
                lookahead_marker_.color.r = 1.0;
                lookahead_marker_.color.g = 0.0;
                lookahead_marker_.color.b = 0.0;
                if (idx_memory % 5 == 0)
                {
                    pub_marker_->publish(lookahead_marker_);
                }
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }
}

// 控制车辆转向使得朝向目标方向
void PurePursuit::goal_orientation_control(const geometry_msgs::msg::Pose& goal_pose, bool *orient_reached)
{
    double goal_yaw = tf2::getYaw(goal_pose.orientation);
    double current_yaw = tf2::getYaw(odom_.pose.pose.orientation);  // 使用里程计信息获得当前 yaw
    double yaw_error = goal_yaw - current_yaw;
    while (yaw_error > M_PI)
        yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI)
        yaw_error += 2 * M_PI;

    *orient_reached = false;
    if (std::abs(yaw_error) > orientation_tolerance_)
    {
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = yaw_error * 1.0;
        cmd_vel_.angular.z = std::min(0.8, std::abs(cmd_vel_.angular.z));
        if (yaw_error < 0)
        {
            cmd_vel_.angular.z = -cmd_vel_.angular.z;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal reached with correct orientation.");
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
        *orient_reached = true;
    }
    cmd_vel_.angular.z = std::isnan(cmd_vel_.angular.z) ? 0.0 : cmd_vel_.angular.z;
    cmd_vel_.angular.z = std::copysign(std::min(std::fabs(cmd_vel_.angular.z), w_max_), cmd_vel_.angular.z);
    pub_vel_->publish(cmd_vel_);
}

// 简单字符串分割函数
std::vector<std::string> PurePursuit::split(const std::string& str, char delimiter)
{
    std::vector<std::string> result;
    std::size_t start = 0;
    std::size_t pos = 0;
    while ((pos = str.find(delimiter, start)) != std::string::npos)
    {
        if (pos != start)
        {
            result.push_back(str.substr(start, pos - start));
        }
        start = pos + 1;
    }
    if (start < str.size())
    {
        result.push_back(str.substr(start));
    }
    return result;
}

// 服务回调：复位路径
bool PurePursuit::reset_path_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                           std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;
    (void)res;
    if (goal_reached_)
    {
        reset_path_en = true;
        RCLCPP_INFO(this->get_logger(), "Reset Path and Continue to run.");
    }
    return true;
}

// 接收路径点消息
void PurePursuit::waypoints_listener(const nav_msgs::msg::Path::SharedPtr new_path)
{
    if (new_path->header.frame_id == map_frame_id_)
    {
        path_ = *new_path;
        idx_ = 0;
        if (!new_path->poses.empty())
        {
            std::cout << "Received Waypoints" << std::endl;
            path_loaded_ = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received empty waypoint!");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
                    "The waypoints must be published in the %s frame! Ignoring path in %s frame!",
                    map_frame_id_.c_str(), new_path->header.frame_id.c_str());
    }
}

// 将全局坐标下的位姿转换到 base_link 坐标系下
KDL::Frame PurePursuit::trans2base(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Transform& tf)
{
    KDL::Frame F_map_pose(
                KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
    KDL::Frame F_map_tf(
                KDL::Rotation::Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
                KDL::Vector(tf.translation.x, tf.translation.y, tf.translation.z));
    return F_map_tf.Inverse() * F_map_pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
