/**
* @file	om_manual.cpp
* @brief 台車をマニュアルで動かせる
* @details
* @attention
* @note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version	Ver.1.00 Dec.13.2023 ryu
    - joyボタンを操作しない場合、常時に0速度を出す処理を追加

* @version	Ver.1.00 Aug.20.2023 ryu
    - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "om_cart/msg/cmd.hpp"
#include "om_cart/om_cart.hpp"

rclcpp::Publisher<om_cart::msg::Cmd>::SharedPtr cart_cmd_pub;

double            VEL_LINE_MAX = 300;     // mm/s
const uint16_t    STEP_CHANGE = 100;      //mm/s
const double      VEL_THETA_MAX = M_PI_4; // pi/4 /s
om_cart::msg::Cmd cmd_msg;
double speed_last_line;
double speed_last_twist;

uint lost_ctrl_wdt_cnt;

void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    static uint cnt = 0;
    lost_ctrl_wdt_cnt = 0;

    //buttons[1]: [A]
    //buttons[2]: [Y]
    if(joy->buttons[1] == 1) //[A]
    {
        RCLCPP_INFO(rclcpp::get_logger("om_manual"), "Entered Emergency Protect Mode!\r\nPress [Y] back to Normal Mode.");
        cmd_msg.type = OPERATE;
        cmd_msg.size = 1;
        cmd_msg.data[0] = S_OFF;
        cart_cmd_pub->publish(cmd_msg);
        return;

    }
    if(joy->buttons[2] == 1) //[Y]
    {
        RCLCPP_INFO(rclcpp::get_logger("om_manual"), "Normal Mode!");
        cmd_msg.type = OPERATE;
        cmd_msg.size = 1;
        cmd_msg.data[0] = S_ON;
        cart_cmd_pub->publish(cmd_msg);
        return;

    }
    if((int)joy->axes[7] != 0)
    {
        VEL_LINE_MAX += (joy->axes[7] * STEP_CHANGE);
        if(VEL_LINE_MAX > 800) VEL_LINE_MAX = 800;
        if(VEL_LINE_MAX < 200) VEL_LINE_MAX = 200;
        return;
    }

    if((abs((int)joy->axes[0] * 100) > 0) || (abs((int)joy->axes[4] * 100) > 0))
    {
        //set cart speed
        cmd_msg.type = SET_SPEED;
        cmd_msg.size = 2;
        cmd_msg.data[0] = (joy->axes[4] * VEL_LINE_MAX) / 1000;    //line speed
        cmd_msg.data[1] = (joy->axes[0] * VEL_THETA_MAX);   //theta speed

        cart_cmd_pub->publish(cmd_msg);
        cnt = 0;

        speed_last_line = cmd_msg.data[0];
        speed_last_twist = cmd_msg.data[1];
    }
    else if(cnt < 3)
    {
        cmd_msg.type = SET_SPEED;
        cmd_msg.size = 2;
        cmd_msg.data[0] = 0;  //line speed
        cmd_msg.data[1] = 0;  //theta speed
        cart_cmd_pub->publish(cmd_msg);
        cnt += 1;
        speed_last_line = cmd_msg.data[0];
        speed_last_twist = cmd_msg.data[1];
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("om_manual");

    cart_cmd_pub = node->create_publisher<om_cart::msg::Cmd>("cart_cmd", 10);
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joyCallback);

    rclcpp::Rate loop_rate(1000);
    lost_ctrl_wdt_cnt = 0;

    while (rclcpp::ok())
    {
        lost_ctrl_wdt_cnt++;
        if (lost_ctrl_wdt_cnt > 100)
        {
            lost_ctrl_wdt_cnt = 0;
            //all joy buttons released
            cmd_msg.type = SET_SPEED;
            cmd_msg.size = 2;
            cmd_msg.data[0] = speed_last_line;  //line speed
            cmd_msg.data[1] = speed_last_twist;  //theta speed
            cart_cmd_pub->publish(cmd_msg);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
