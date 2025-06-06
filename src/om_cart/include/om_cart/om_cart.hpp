/**
* @file    om_cart.h
* @brief   Receive and publish motor information (ROS2 version)
* @details
* @attention
* @note
* @version Ver.2.00 Dec.13.2024 ryu
    - Converted to ROS2 format

* @version Ver.1.00 Aug.20.2023 ryu
    - Initial creation in ROS1 format
*/

#ifndef OM_CART_H
#define OM_CART_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>

#include "om_msgs/msg/query.hpp"
#include "om_msgs/msg/response.hpp"
#include "om_msgs/msg/state.hpp"
#include "om_data.hpp"

enum
{
    WHEEL_LEFT_ID  = 1,
    WHEEL_RIGHT_ID = 2,
    CART_ID        = 15,
};

enum
{
    S_OFF   = 0,
    S_ON    = 1,
    ALM_RST = 2,
};

enum
{
    FC_READ       = 3,  //0x03
    FC_WRITE      = 16, //0x10
    FC_READ_WRITE = 23  //0x17
};

enum
{
    UN_USED         = 0,
    OPERATE         = 1,
    SET_SPEED       = 2,
    SET_MOTOR_PARAM = 3,
    SET_CART_PARAM  = 4,
};

enum
{
    CART_STATUS = 1,
    COMM_RESULT = 2,
};

typedef union
{
     struct
     {
        double vel_line;              //0
        double vel_theta;             //1
        double vel_left;              //2
        double vel_right;             //3
        double alm_code_L;            //4
        double alm_code_R;            //5
        double main_power_volt_L;     //6
        double main_power_volt_R;     //7
        double main_power_curr_L;     //8
        double main_power_curr_R;     //9
        double motor_temp_L;          //10
        double motor_temp_R;          //11
        double driver_temp_L;         //12
        double driver_temp_R;         //13
        double emergen_stop;          //14
     } cart_status;

     struct
     {
        double result;
        double error_code;
     }  comm_status;

     double data[30];

} Cart_Status_Info;

const uint8_t CART_STATUS_INFO_SIZE = 30;//14;

typedef union
{
    struct
    {
        double vel_line;
        double vel_theta;
    } set_speed;

    struct
    {
        double set_cmd;
    } operate_cmd;

    double data[30];

} Cart_Cmd_Info;

typedef struct
{
    int32_t vel;                //word[0]
    int32_t alm_code;           //word[1]
    int32_t main_volt;          //word[2]
    int32_t main_curr;          //word[3]
    int32_t motor_temp;         //word[4]
    int32_t driver_temp;        //word[5]
    int32_t driver_IO_INPUT;    //word[6]
    int32_t driver_IO_OUTPUT;   //word[7]
    int32_t reserved_0;         //word[8]
    int32_t reserved_1;         //word[9]
    int32_t reserved_2;         //word[10]
    int32_t reserved_3;         //word[11]
} Motor_Info;

typedef union
{
    struct
    {
        Motor_Info left;
        Motor_Info right;
    } Motor_Status;

    int32_t data[30];
} Cart_Resp_Info;
const uint8_t CART_RESP_INFO_ITEM_SIZE = 12 * 2; //6 * 2; // Left,Right wheel

const uint16_t UNIT_RATIO = 100;
double CART_TREAD;

#endif // OM_CART_H
