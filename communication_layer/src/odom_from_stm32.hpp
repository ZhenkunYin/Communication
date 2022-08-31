#ifndef __ODOM_FROM_STM32__
#define __ODOM_FROM_STM32__

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "serial/serial.h"

#include <thread>
#include <unistd.h>

class odom_publisher
{
private:
    /* data */
    nav_msgs::Odometry odom_data;
    geometry_msgs::TransformStamped odom2base_data;

    tf2::Quaternion q;

    double x;
    double y;
    double th;

    double vx;
    double vy;
    double vth;

    double dt;

    ros::Time odom_Current, odom_last;

public:
    /* data */
    serial::Serial sp; //serial port
    bool state;

    odom_publisher(/* args */);
    ~odom_publisher();

    nav_msgs::Odometry get_odom_data();
    geometry_msgs::TransformStamped get_odom2base_data();
    void update_odom_info(uint8_t *buf, int num);
    void update_gimbal_info(uint8_t *buf, int num);
};
#endif