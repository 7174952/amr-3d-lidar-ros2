#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class OdomSelector : public rclcpp::Node
{
public:
    OdomSelector()
    : Node("odom_selector"), gps_cov_threshold_(5.0)
    {
        // 声明参数并读取
        this->declare_parameter<double>("gps_cov_threshold", 0.5);
        this->get_parameter("gps_cov_threshold", gps_cov_threshold_);
        this->declare_parameter<bool>("enable_gnss", false);
        this->get_parameter("enable_gnss", enable_gnss);

        RCLCPP_INFO(this->get_logger(), "📏 GPS 协方差阈值设为: %.2f", gps_cov_threshold_);
        map_frame_ = "map";
        base_frame_ = "base_link";
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 订阅
        sub_hdl_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/hdl_localization/odom", 10,
            std::bind(&OdomSelector::hdlCallback, this, std::placeholders::_1));

        sub_gps_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/gps", 10,
            std::bind(&OdomSelector::gpsCallback, this, std::placeholders::_1));
        sub_start_turn_round_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_turn_round",10,
            std::bind(&OdomSelector::startTurnRoundCallback, this, std::placeholders::_1));

        // 发布
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8);

        RCLCPP_INFO(this->get_logger(), "✅ Odom Selector started.");

    }

private:
    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_hdl_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_turn_round_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

    // 最新消息存储
    nav_msgs::msg::Odometry selected_msg;
    nav_msgs::msg::Odometry::SharedPtr last_hdl_msg_;
    nav_msgs::msg::Odometry::SharedPtr last_gps_msg_;
    nav_msgs::msg::Odometry::SharedPtr prev_gps_msg_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string map_frame_;
    std::string base_frame_;

    // 协方差阈值
    double gps_cov_threshold_;
    bool enable_gnss = false;
    bool use_gps_ = false;
    double avg_cov = 10000.0;
    double pre_avg_cov = 10000.0;

    int gps_good_cnt_ = 0;
    const int gps_good_cnt_threshold_ = 50; // 例如10帧=1秒

    bool is_start_turn_round = false;
    nav_msgs::msg::Odometry lidar_yaw;


    void startTurnRoundCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_start_turn_round = msg->data;
    }

    void hdlCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        lidar_yaw = *msg;
        if(enable_gnss && use_gps_)
            return;

        selected_msg = *msg;
        publishOdom();
    }

    void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (last_gps_msg_)
            prev_gps_msg_ = last_gps_msg_;
        last_gps_msg_ = msg;
        tryPublishGpsOdom();
    }

    void tryPublishGpsOdom()
    {
        static double last_yaw = 0.0;

        // 计算GPS协方差
        avg_cov = std::max(last_gps_msg_->pose.covariance[0], last_gps_msg_->pose.covariance[8]);

        // 判断是否满足连续若干帧都满足阈值
        if (avg_cov < gps_cov_threshold_)
        {
            gps_good_cnt_++;
            if (!use_gps_ && (gps_good_cnt_ >= gps_good_cnt_threshold_))
            {
                use_gps_ = true;
                RCLCPP_INFO(this->get_logger(), "切换到GPS里程计！");
            }
        }
        else
        {
            gps_good_cnt_ = 0;
            if (use_gps_ && (avg_cov > gps_cov_threshold_ * 2.0))
            {
                use_gps_ = false;
                RCLCPP_INFO(this->get_logger(), "GPS协方差过大，回退到激光里程计！");

                //Reinit robot pose by gnss datas
                geometry_msgs::msg::PoseWithCovarianceStamped initPose_msg;
                initPose_msg.pose.pose.position = last_gps_msg_->pose.pose.position;
                tf2::Quaternion q;
                q.setRPY(0, 0, last_yaw);
                q.normalize();
                initPose_msg.pose.pose.orientation = tf2::toMsg(q);
                initialpose_pub_->publish(initPose_msg);
                RCLCPP_INFO(this->get_logger(), "Switch to lidar and init Robot pose");

            }
        }

        if(is_start_turn_round) //起点终点时借用lidar的方向数据判断方向
        {
            selected_msg = *last_gps_msg_;
            selected_msg.pose.pose.orientation = lidar_yaw.pose.pose.orientation;
            publishOdom();
        }
        else if(use_gps_ && prev_gps_msg_)   // 使用当前策略
        {
            selected_msg = *last_gps_msg_;

            const auto &p1 = prev_gps_msg_->pose.pose.position;
            const auto &p2 = last_gps_msg_->pose.pose.position;

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double dist = std::hypot(dx, dy);

            double yaw;
            if (dist > 0.05)
            {
                yaw = std::atan2(dy, dx);
                last_yaw = yaw;
            }
            else
            {
                yaw = last_yaw;
            }

            //
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            q.normalize();
            selected_msg.pose.pose.orientation = tf2::toMsg(q);

            publishOdom();
        }

        pre_avg_cov = avg_cov;

    }

    void publishOdom()
    {
        selected_msg.header.stamp = this->now();
        selected_msg.header.frame_id = "odom";
        pub_odom_->publish(selected_msg);

        //publish tf map->base_link
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = selected_msg.header.stamp;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = base_frame_;

        tf_msg.transform.translation.x = selected_msg.pose.pose.position.x;
        tf_msg.transform.translation.y = selected_msg.pose.pose.position.y;
        tf_msg.transform.translation.z = selected_msg.pose.pose.position.z;

        tf_msg.transform.rotation = selected_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSelector>());
    rclcpp::shutdown();
    return 0;
}
