// -------------------- 标准C/C++库 --------------------
#include <cmath>      
#include <cstdio>     
#include <iostream>   
#include <string>     

// -------------------- ROS核心库 --------------------
#include <ros/ros.h>  // ROS系统核心功能

// -------------------- ROS消息类型 --------------------
// 几何相关消息
#include <geometry_msgs/PoseStamped.h>      // 带时间戳的位姿 (位置+姿态)
#include <geometry_msgs/Twist.h>            // 线速度和角速度
#include <geometry_msgs/TwistStamped.h>     // 带时间戳的Twist

// MAVROS特定消息
#include <mavros_msgs/CommandBool.h>        // 用于解锁/上锁无人机
#include <mavros_msgs/CommandLong.h>        // 用于发送长命令（如触发特定功能）
#include <mavros_msgs/PositionTarget.h>     // 位置目标设定
#include <mavros_msgs/SetMode.h>            // 用于切换飞行模式
#include <mavros_msgs/State.h>              // 包含飞控当前状态（模式、是否解锁等）

// 标准消息
#include <nav_msgs/Odometry.h>              // 里程计信息（位置、速度、姿态）
#include <std_msgs/Bool.h>                  // 布尔值消息
#include <std_msgs/Int32.h>                 // 32位整数消息
#include <std_msgs/String.h>                // 字符串消息

// -------------------- ROS功能库 --------------------
#include <serial/serial.h>                  // 串口通信功能
#include <tf/transform_listener.h>          // TF坐标变换监听器


#define ALTITUDE  1.0

int flag = 100;// 100表示未收到串口指令

/*[x,y]代表第x行第y列的坐标，flag=xy
例如：flag=12表示第1行第2列的坐标

flag=99表示开始任务
68代表起飞点
flag == 93 代表开始
flag == 92 代表以AUTO.LAND模式降落
flag == 91 代表以45度方式降落
*/

void uartcallback(const std_msgs::Int32::ConstPtr& msg)
{
    flag = msg->data;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//定义变量，用于接收无人机的里程计和姿态信息
tf::Quaternion quat; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
bool  flag_init_position = false;
ros::Time last_request;

nav_msgs::Odometry local_pos;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
//回调函数接收无人机的里程计和姿态信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    //程序启动后获取一次最新的GPS和气压计漂移数据作为初始值
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;          
    }
    //四元数转为欧拉角，后续程序使用
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);   
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);


}
int main(int argc, char **argv)
{
    //初始化节点，ROSAPI接口
    ros::init(argc, argv, "offboard_single_position");
    
    //创建nh句柄，ROSAPI接口
    ros::NodeHandle nh;
  
    //创建订阅者，用于订阅无人机的当前飞行状态等信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    //创建发布者，发布无人机的期望位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    //创建订阅者，订阅串口数据
    ros::Subscriber uart_sub = nh.subscribe("/uart_data", 10, uartcallback);
    
    //创建订阅者，订阅无人机的实时位置信息
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    //创建无人机解锁客户端，用于想飞控请求解锁命令
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
 
    //创建模式切换客户端，用于ROS程序想底层飞控请求进入offbaord或者其他模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //offboard模式下，需要保持2Hz以上频率的心跳包，此处设置为20Hz，可适当调整
    ros::Rate rate(20.0);

    //等待连接飞控，连接后再执行后续部分
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //设置预发布位置，有了预发布位置，才能切入到offboard模式
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =init_position_x_take_off + 0;
    pose.pose.position.y =init_position_y_take_off + 0;
    pose.pose.position.z =init_position_z_take_off + ALTITUDE;
     
    //发布者发布期望位置
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

        while(ros::ok() && flag != 99)
    {   
        ROS_INFO("Waiting for start command (flag=99)...");
        ros::spinOnce();
        rate.sleep();
    }
        
        //模式切换变量定义
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
    
    //解锁变量定义
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

    //获取系统当前时间给变量last_request
        ros::Time start_time  = ros::Time::now();
    
    while(ros::ok())
            {
                //请求进入OFFBOARD模式，每隔5秒请求一次
                if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    {
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                    start_time = ros::Time::now();
                    flag_init_position = false;         
                }
                else 
                {
                    //请求解锁，每隔5秒请求一次
                    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                    {
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                        start_time = ros::Time::now();
                        flag_init_position = false;         
                    }
                }
                
                //2、添加时间判断，使得无人机跳出模式切换循环
                if(ros::Time::now() - last_request > ros::Duration(8.0))
                {
                    break;
                    flag = 68;
                }
                    
                //发布期望位置信息
                pose.pose.position.x =init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }       
            last_request = ros::Time::now();        

        while(ros::ok())
        {

            ROS_INFO("target_X: %f target_Y: %f",pose.pose.position.x,pose.pose.position.y);
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
            ROS_WARN("%d",flag);

            if(flag == 68 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 67 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 0.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 66 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 1;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 65 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 1.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if(flag == 64 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 2;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 63 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 2.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 62 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 3;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 61 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 3.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 60 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 4;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 58 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 48 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 38 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 28 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 18 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 8 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 3;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if(flag == 57 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 0.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 47 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1;
                pose.pose.position.y =init_position_y_take_off + 0.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 37 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 0.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 27 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2;
                pose.pose.position.y =init_position_y_take_off + 0.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if(flag == 17 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 0.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 7 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 3;
                pose.pose.position.y =init_position_y_take_off + 0.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 56 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 1;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if(flag == 46 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1;
                pose.pose.position.y =init_position_y_take_off + 1;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 36 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 1;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if(flag == 26 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2;
                pose.pose.position.y =init_position_y_take_off + 1;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if(flag == 16 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 1;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 6 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 3;
                pose.pose.position.y =init_position_y_take_off + 1;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 55 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 1.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 45 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 1.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 35 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 1.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 25 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.0;
                pose.pose.position.y =init_position_y_take_off + 1.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if (flag == 15 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 1.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if (flag == 5 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 1.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 54 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 2;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 44 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1;
                pose.pose.position.y =init_position_y_take_off + 2;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 34 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 2;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }   

            if (flag == 24 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2;
                pose.pose.position.y =init_position_y_take_off + 2;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 14 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 2;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 4 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 3;
                pose.pose.position.y =init_position_y_take_off + 2;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 53 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 2.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 43 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 2.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 33 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 2.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 23 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.0;
                pose.pose.position.y =init_position_y_take_off + 2.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 13 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 2.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 3 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 2.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 52 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 1.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 42 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 1.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 32 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 1.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 22 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.0;
                pose.pose.position.y =init_position_y_take_off + 1.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 12 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 1.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 2 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 1.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 51 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 3.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 41 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 3.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 31 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 3.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if (flag == 21 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.0;
                pose.pose.position.y =init_position_y_take_off + 3.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 11 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 3.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 1 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 3.5;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 50 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 0.5;
                pose.pose.position.y =init_position_y_take_off + 4.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 40 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 4.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if (flag == 30 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.5;
                pose.pose.position.y =init_position_y_take_off + 4.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 20 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.0;
                pose.pose.position.y =init_position_y_take_off + 4.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 10 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 2.5;
                pose.pose.position.y =init_position_y_take_off + 4.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }
            if (flag == 0 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                pose.pose.position.x = init_position_x_take_off + 1.0;
                pose.pose.position.y =init_position_y_take_off + 4.0;
                pose.pose.position.z =init_position_z_take_off + ALTITUDE;
                last_request = ros::Time::now();
            }

            if (flag == 91 && ros::Time::now() - last_request > ros::Duration(1.0))
            {   
                pose.pose.position.x = init_position_x_take_off + 0;
                pose.pose.position.y =init_position_y_take_off + 0;
                pose.pose.position.z =init_position_z_take_off + 0.3;
            }
            if (flag == 92 && ros::Time::now() - last_request > ros::Duration(8.0))
            {
                ROS_INFO("LAND");
                last_request = ros::Time::now();
                mavros_msgs::SetMode set_mode;
                set_mode.request.custom_mode = "AUTO.LAND";
                if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    ROS_INFO("Auto land sent");
                }
                flag = 93;
            }

            if (flag == 93 && ros::Time::now() - last_request > ros::Duration(1.0))
            {
                break;
                flag = 99; //end of the mission
            }
            

        }

    
    return 0;
}
