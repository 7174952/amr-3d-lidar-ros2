/**
* @file    om_cart.cpp
* @brief   Receive and publish motor information (ROS2 version)
* @details
* @attention
* @note
* @version Ver.2.00 Dec.13.2024 ryu
    - Converted to ROS2 format

* @version Ver.1.00 Aug.20.2023 ryu
    - Initial creation in ROS1 format
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <queue>

#include "om_msgs/msg/query.hpp"
#include "om_msgs/msg/response.hpp"
#include "om_msgs/msg/state.hpp"
#include "om_cart/msg/cmd.hpp"
#include "om_cart/msg/status.hpp"
#include "om_cart/om_data.hpp"
#include "om_cart/om_cart.hpp"

geometry_msgs::msg::TwistStamped msg_velocity;
/* Global Variables */
int gState_driver = 0;  /* Communication flag (0: available, 1: busy) */

Om_data om_data;

rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cart_velocity_pub;
rclcpp::Publisher<om_msgs::msg::Query>::SharedPtr om_query_pub;
rclcpp::Publisher<om_cart::msg::Status>::SharedPtr cart_status_pub;

// Publishers for AMR info for jsk_rviz_plugins
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cart_bat_volt_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cart_vel_linear_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cart_vel_radius_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cart_info_rviz_pub;

std::queue<om_cart::msg::Cmd> cmd_msg_buf;
double vel_wait_timer = 0; // Ensure stop when no command arrives
const uint32_t MAX_WAIT_TIME = 50; // 50 => 0.5s
rclcpp::Node::SharedPtr node;

// Function Prototypes
void om_state_callback(const om_msgs::msg::State::SharedPtr msg);
void om_resp_callback(const om_msgs::msg::Response::SharedPtr msg);
void drive_cmd_vel_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
void drive_cmd_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
void cart_auto_drive_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
void wait();
void update();

void cart_auto_drive_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    om_cart::msg::Cmd cmd_tmp;
    cmd_tmp.type = SET_SPEED;
    cmd_tmp.size = 2;
    cmd_tmp.data[0] = msg->linear.x;
    cmd_tmp.data[1] = msg->angular.z;

    cmd_msg_buf.push(cmd_tmp);

    //watchdog for cmd_vel
    vel_wait_timer = 0;
}

void cart_s_on_callback(const std_msgs::msg::Bool::SharedPtr state)
{
    om_cart::msg::Cmd cmd_tmp;
    cmd_tmp.type = OPERATE;
    cmd_tmp.size = 1;
    cmd_tmp.data[0] = (state->data == false) ? S_OFF : S_ON;

    cmd_msg_buf.push(cmd_tmp);

}

void om_state_callback(const om_msgs::msg::State::SharedPtr msg)
{
    gState_driver = msg->state_driver;

}

void om_resp_callback(const om_msgs::msg::Response::SharedPtr msg)
{
    om_cart::msg::Status cart_status_msg;
    Cart_Status_Info cart_status_info;
    for(int i=0; i < CART_STATUS_INFO_SIZE; i++)
    {
        cart_status_info.data[i] = 0;
    }

    double vel_left, vel_right, vel_line, vel_theta;
    std_msgs::msg::Float32 cart_val_tmp;
    std_msgs::msg::String cart_info_rviz_tmp;

    switch( msg->func_code)
    {
        case FC_READ: //Cart status
            {
                Cart_Resp_Info resp_info;

                for(int8_t i = 0; i < CART_RESP_INFO_ITEM_SIZE; i++)
                {
                    resp_info.data[i] = msg->data[i];
                }

                vel_left =  (double)resp_info.Motor_Status.left.vel / UNIT_RATIO;
                vel_right = -(double)resp_info.Motor_Status.right.vel / UNIT_RATIO;
                vel_line = ((vel_left + vel_right) / 2)/1000;
                vel_theta = (vel_right - vel_left) / CART_TREAD;

                cart_status_info.cart_status.vel_line = vel_line;
                cart_status_info.cart_status.vel_theta = vel_theta;
                cart_status_info.cart_status.vel_left = vel_left;
                cart_status_info.cart_status.vel_right = vel_right;

                if((vel_wait_timer > MAX_WAIT_TIME) && ((std::fabs(vel_left) > 0.05) || (std::fabs(vel_right) > 0.05)))
                {
                    std::cout << "om_cart: Force to Stop AMR!" << std::endl;
                    //Force cart to stop
                    om_cart::msg::Cmd cmd_tmp;
                    cmd_tmp.type = SET_SPEED;
                    cmd_tmp.size = 2;
                    cmd_tmp.data[0] = 0.0;
                    cmd_tmp.data[1] = 0.0;

                    cmd_msg_buf.push(cmd_tmp);
                }

                cart_status_info.cart_status.alm_code_L = resp_info.Motor_Status.left.alm_code;
                cart_status_info.cart_status.alm_code_R = resp_info.Motor_Status.right.alm_code;
                cart_status_info.cart_status.main_power_volt_L = resp_info.Motor_Status.left.main_volt;
                cart_status_info.cart_status.main_power_volt_R = resp_info.Motor_Status.right.main_volt;
                cart_status_info.cart_status.main_power_curr_L = resp_info.Motor_Status.left.main_curr;
                cart_status_info.cart_status.main_power_curr_R = resp_info.Motor_Status.right.main_curr;
                cart_status_info.cart_status.motor_temp_L = resp_info.Motor_Status.left.motor_temp;
                cart_status_info.cart_status.motor_temp_R = resp_info.Motor_Status.right.motor_temp;
                cart_status_info.cart_status.driver_temp_L = resp_info.Motor_Status.left.driver_temp;
                cart_status_info.cart_status.driver_temp_R = resp_info.Motor_Status.right.driver_temp;
                cart_status_info.cart_status.emergen_stop = ((resp_info.Motor_Status.left.driver_IO_OUTPUT >> 5) & 0x00000001)
                                                          + ((resp_info.Motor_Status.right.driver_IO_OUTPUT >> 5) & 0x00000001);
                cart_status_msg.type = CART_STATUS;
                cart_status_msg.size = CART_STATUS_INFO_SIZE;

                for(int8_t i = 0; i < CART_STATUS_INFO_SIZE; i++)
                {
                    cart_status_msg.data[i] = cart_status_info.data[i];
                }
                cart_status_pub->publish(cart_status_msg);

                //make and send velocity

                msg_velocity.header.stamp    = node->now();
                msg_velocity.header.frame_id = "";
                msg_velocity.twist.linear.x  = vel_line;
                msg_velocity.twist.linear.y  = 0;
                msg_velocity.twist.angular.z = vel_theta;
                cart_velocity_pub->publish(msg_velocity);

                //To show on rviz by jsk_visualize_plugins
                cart_val_tmp.data = (cart_status_info.cart_status.main_power_volt_L/10 + cart_status_info.cart_status.main_power_volt_R/10)/2;
                cart_bat_volt_pub->publish(cart_val_tmp);
                cart_val_tmp.data = vel_line;
                cart_vel_linear_pub->publish(cart_val_tmp);
                cart_val_tmp.data = vel_theta;
                cart_vel_radius_pub->publish(cart_val_tmp);

                cart_info_rviz_tmp.data = "Normal";
                if((uint32_t)(cart_status_info.cart_status.alm_code_L != 0) || (uint32_t)(cart_status_info.cart_status.alm_code_R != 0))
                {
                    cart_info_rviz_tmp.data = "alm_code_left:" + std::to_string((uint32_t)(cart_status_info.cart_status.alm_code_L)) + "\n"
                                              "alm_code_right:" + std::to_string((uint32_t)(cart_status_info.cart_status.alm_code_R));
                }
                cart_info_rviz_pub->publish(cart_info_rviz_tmp);
            }
            break;
        case FC_WRITE:
            cart_status_msg.type = COMM_RESULT;
            cart_status_msg.size = 2;
            cart_status_msg.data[0] = 1;
            cart_status_msg.data[1] = msg->func_code;
            cart_status_pub->publish(cart_status_msg);
            break;
        case FC_READ_WRITE:

            break;
        default:
            break;
    }

}

void cart_set_cmd_callback(const om_cart::msg::Cmd::SharedPtr msg)
{
    vel_wait_timer = 0;

    cmd_msg_buf.push(*msg);
}

//---------------------------------------------------------------------------
//   処理待ちサービス関数
//
//@details	規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス
//----------------------------------------------------------------------------
void wait()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    rclcpp::spin_some(node->get_node_base_interface());

    /* ドライバの通信が終了するまでループ */
    while(gState_driver == 1)
    {
        rclcpp::spin_some(node->get_node_base_interface());
    }
}

void update()
{
    if(cmd_msg_buf.size() > 0)
    {
        om_cart::msg::Cmd msg = cmd_msg_buf.front();

        //check
        switch (msg.type)
        {
            case OPERATE:
                switch((uint)msg.data[0])
                {
                    case S_OFF:
                        om_data.drive_wheel_end(CART_ID, om_query_pub);
                        break;
                    case S_ON:
                        om_data.drive_wheel_begin(CART_ID, om_query_pub);
                        break;
                    case ALM_RST:
                    default:
                        om_data.reset_alarm(WHEEL_LEFT_ID, 0, om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_LEFT_ID, 1, om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_LEFT_ID, 0, om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_RIGHT_ID, 0, om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_RIGHT_ID, 1, om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_RIGHT_ID, 0, om_query_pub);
                        break;
                }

                break;
            case SET_SPEED: //update motor speed
                Cart_Cmd_Info cmd_info;

                for(uint8_t i = 0; i < msg.size; i++)
                {
                    cmd_info.data[i] = msg.data[i];
                }

                om_data.drive_cart_cmd(CART_ID, cmd_info.set_speed.vel_line*100000, cmd_info.set_speed.vel_theta*100, om_query_pub);
                wait();
                om_data.update_share_state(CART_ID, om_query_pub);

                break;
            case SET_MOTOR_PARAM: //TBD

                break;
            case SET_CART_PARAM: //TBD

                break;
            case UN_USED:
            default: //Do nothing

                break;
        }
        cmd_msg_buf.pop();
    }
    else
    {
        om_data.update_share_state(CART_ID, om_query_pub);
    }

    wait();

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("om_cart_node");

    // Subscribers
    auto  om_state_sub = node->create_subscription<om_msgs::msg::State>(
        "om_state", 100, std::bind(om_state_callback, std::placeholders::_1));

    auto om_resp_sub = node->create_subscription<om_msgs::msg::Response>(
        "om_response", 100, std::bind(om_resp_callback, std::placeholders::_1));

    auto cart_set_cmd_sub = node->create_subscription<om_cart::msg::Cmd>(
        "cart_cmd", 1, std::bind(cart_set_cmd_callback, std::placeholders::_1));


    auto cart_auto_drive_cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cart_auto_drive_cmd", 1, std::bind(cart_auto_drive_cmd_callback, std::placeholders::_1));

    auto cart_s_on_sub = node->create_subscription<std_msgs::msg::Bool>(
        "cart_s_on", 1, std::bind(cart_s_on_callback, std::placeholders::_1));

    // Publishers
    cart_status_pub = node->create_publisher<om_cart::msg::Status>("cart_status", 100);
    om_query_pub = node->create_publisher<om_msgs::msg::Query>("om_query1", 1);
    cart_velocity_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("vehicle_vel", 100);

    // Publishers for AMR info for jsk_rviz_plugins
    cart_bat_volt_pub = node->create_publisher<std_msgs::msg::Float32>("cart_vbat", 10);
    cart_vel_linear_pub = node->create_publisher<std_msgs::msg::Float32>("vel_linear", 10);
    cart_vel_radius_pub = node->create_publisher<std_msgs::msg::Float32>("vel_radius", 10);
    cart_info_rviz_pub = node->create_publisher<std_msgs::msg::String>("cart_info", 10);

    // Initialize velocity publisher
    rclcpp::Time init_time = node->now();
    msg_velocity.header.stamp = init_time;
    msg_velocity.header.frame_id = "";
    msg_velocity.twist.linear.x = 0;
    msg_velocity.twist.linear.y = 0;
    msg_velocity.twist.angular.z = 0;

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Configure all shared registers
    RCLCPP_INFO(node->get_logger(), "Init to config Share register!");

    om_data.set_share_ID(om_data.slave_1_id, 1, om_query_pub);
    wait();
    om_data.set_share_ID(om_data.slave_2_id, 2, om_query_pub);
    wait();
    om_data.set_share_net_id(om_data.slave_1_id, om_query_pub);
    wait();
    om_data.set_share_net_id(om_data.slave_2_id, om_query_pub);
    wait();

    // Start to run motor
    RCLCPP_INFO(node->get_logger(), "Start to run motor!");
    om_data.drive_wheel_begin(om_data.global_id, om_query_pub);
    wait();

    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok())
    {
        vel_wait_timer++;
        update();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // Stop motor
    RCLCPP_INFO(node->get_logger(), "Stop Motor!");

    RCLCPP_INFO(node->get_logger(), "End and exit!");

    rclcpp::shutdown();
    return 0;
}
