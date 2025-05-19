#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsCorrectorNode : public rclcpp::Node
{
public:
    GpsCorrectorNode()
    : Node("gps_corrector_node")
    {
        // 声明参数
        this->declare_parameter<double>("gps_altitude_offset", 0.8);
        this->get_parameter("gps_altitude_offset", gps_altitude_offset_);

        RCLCPP_INFO(this->get_logger(), "🛰️ 海拔修正偏移量为: %.2f m", gps_altitude_offset_);

        // 订阅 /fix_raw
        sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix_raw", 10,
            std::bind(&GpsCorrectorNode::callback, this, std::placeholders::_1)
        );

        // 发布 /fix
        pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
    }

private:
    void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        auto corrected = *msg;  // 拷贝原始数据

        // 修正高度
        corrected.altitude -= gps_altitude_offset_;

        pub_->publish(corrected);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
    double gps_altitude_offset_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsCorrectorNode>());
    rclcpp::shutdown();
    return 0;
}
