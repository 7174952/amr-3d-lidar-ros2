#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10,
            std::bind(&OdomSelector::imuCallback, this, std::placeholders::_1));

        // 发布
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        RCLCPP_INFO(this->get_logger(), "✅ Odom Selector started.");

        imu_offset_total = 0;
    }

private:
    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_hdl_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

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

    rclcpp::Time last_gps_time_;
    geometry_msgs::msg::Point last_gps_pos_;
    geometry_msgs::msg::Point prev_gps_pos_;
    double integrated_yaw_ = 0.0;
    rclcpp::Time last_imu_time_;
    double imu_offset_z;
    double imu_offset_total;
    int imu_cnt = 0;
    bool gps_ready_ = false;

    void hdlCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(enable_gnss && (pre_avg_cov < gps_cov_threshold_))
            return;

        selected_msg = *msg;
        publishOdom();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(enable_gnss && (pre_avg_cov < gps_cov_threshold_))
        {
            if (!gps_ready_) return;

            // 时间间隔
            rclcpp::Time imu_time = msg->header.stamp;
            if (last_imu_time_.nanoseconds() == 0)
            {
                last_imu_time_ = imu_time;
                return;
            }
            double dt = (imu_time - last_imu_time_).seconds();
            last_imu_time_ = imu_time;

            // 积分yaw（假设只考虑 z 轴角速度）
            integrated_yaw_ += (msg->angular_velocity.z - imu_offset_z) * dt;

            // 插值位置
            double gps_dt = (imu_time - last_gps_time_).seconds();
            double total_gps_dt = (last_gps_time_ - rclcpp::Time(last_gps_msg_->header.stamp)).seconds();
            if (total_gps_dt <= 0.0) return;

            double ratio = std::min(std::max(gps_dt / total_gps_dt, 0.0), 1.0);
            geometry_msgs::msg::Point interp_pos;
            interp_pos.x = prev_gps_pos_.x + (last_gps_pos_.x - prev_gps_pos_.x) * ratio;
            interp_pos.y = prev_gps_pos_.y + (last_gps_pos_.y - prev_gps_pos_.y) * ratio;
            interp_pos.z = prev_gps_pos_.z + (last_gps_pos_.z - prev_gps_pos_.z) * ratio;

            // 生成并发布 Odometry
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = imu_time;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position = interp_pos;

            tf2::Quaternion q;
            q.setRPY(0, 0, integrated_yaw_);
            q.normalize();
            odom_msg.pose.pose.orientation = tf2::toMsg(q);

            pub_odom_->publish(odom_msg);

            // 可选: 发送 TF
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = odom_msg.header;
            tf_msg.child_frame_id = base_frame_;
            tf_msg.transform.translation.x = interp_pos.x;
            tf_msg.transform.translation.y = interp_pos.y;
            tf_msg.transform.translation.z = interp_pos.z;
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }

    }

    void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (last_gps_msg_)
            prev_gps_msg_ = last_gps_msg_;
        last_gps_msg_ = msg;
        tryPublishGpsOdom();

        last_gps_time_ = last_gps_msg_->header.stamp;
        prev_gps_pos_ = last_gps_pos_;
        last_gps_pos_ = last_gps_msg_->pose.pose.position;
        // integrated_yaw_ = 0.0;
        gps_ready_ = true;
    }

    void tryPublishGpsOdom()
    {
        static double last_yaw = 0.0;

        // 计算GPS协方差
        avg_cov = last_gps_msg_->pose.covariance[0];

        // 判断是否切换来源
        if (avg_cov < gps_cov_threshold_)
        {
            use_gps_ = true;
        }
        else if (avg_cov > gps_cov_threshold_ * 2.0)
        {
            use_gps_ = false;
        }

        // 使用当前策略
        if (use_gps_ && prev_gps_msg_)
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
            integrated_yaw_ = yaw;

            //static transform gps->base_link
            double x_raw = selected_msg.pose.pose.position.x;
            double y_raw = selected_msg.pose.pose.position.y;
            double z_raw = selected_msg.pose.pose.position.z;

            selected_msg.pose.pose.position.x = x_raw;
            selected_msg.pose.pose.position.y = y_raw;
            selected_msg.pose.pose.position.z = z_raw;

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
