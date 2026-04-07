#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <string>
#include <cstdio>

void uartcallback(const std_msgs::Int32::ConstPtr& msg)
{
    int flag = msg->data;
    ROS_INFO("%d", flag);
}

int x , y;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "uart_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("uart_data", 10);
    ros::Subscriber uart_sub = nh.subscribe("uart_data" , 1000, uartcallback);
    
    serial::Serial ser;
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(100); // 50 Hz

    while(ros::ok())
    {
        if(ser.available())
        {
            std_msgs::String str;
            std_msgs::Int32 msg;
            str.data = ser.read(ser.available());
            int items_parsed = sscanf(str.data.c_str(), "[%d,%d]", &x, &y);
            if(items_parsed == 2)
            {
                    msg.data = 10*x + y;
                    pub.publish(msg);
                    ROS_INFO_STREAM("msg: " << msg.data << " x: " << x << " y: " << y);
                    
            }
        }
        ros::spinOnce();
    }
    
    ser.close();
    return 0;
}