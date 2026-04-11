// -------------------- 标准C/C++库 --------------------
#include <cmath>      // 用于数学计算，如 sin(), cos()
#include <cstdio>     // 用于C风格的输入输出，如 printf()
#include <iostream>   // 用于C++风格的输入输出，如 std::cout
#include <string>     // 用于 std::string 类

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

// 新增函数：根据flag值设置目标位置
void setTargetPosition(int flag_val, geometry_msgs::PoseStamped& pose, 
                      float init_x, float init_y, float init_z)
{
    // 特殊flag值处理
    if (flag_val == 91)
    {
        pose.pose.position.x = init_x;
        pose.pose.position.y = init_y;
        pose.pose.position.z = init_z + 0.3;
        return;
    }
    
    // 普通flag值处理：解析十位和个位
    if (flag_val >= 0 && flag_val <= 99)
    {
        int row = flag_val / 10;  // 十位数
        int col = flag_val % 10;  // 个位数
        
        // 根据行和列计算目标位置
        // 行号对应Y轴偏移：0->0, 1->0.5, 2->1.0, ..., 6->3.5, 7->4.0
        // 列号对应X轴偏移：0->0, 1->0.5, 2->1.0, ..., 6->3.5, 7->4.0
        
        // 特殊处理：某些位置有固定偏移
        float x_offset = 0.0;
        float y_offset = 0.0;
        
        // 计算X轴偏移
        if (col == 0) x_offset = 0.0;
        else if (col == 1) x_offset = 0.5;
        else if (col == 2) x_offset = 1.0;
        else if (col == 3) x_offset = 1.5;
        else if (col == 4) x_offset = 2.0;
        else if (col == 5) x_offset = 2.5;
        else if (col == 6) x_offset = 3.0;
        else if (col == 7) x_offset = 3.5;
        else if (col == 8) x_offset = 4.0;
        
        // 计算Y轴偏移
        if (row == 0) y_offset = 0.0;
        else if (row == 1) y_offset = 0.5;
        else if (row == 2) y_offset = 1.0;
        else if (row == 3) y_offset = 1.5;
        else if (row == 4) y_offset = 2.0;
        else if (row == 5) y_offset = 2.5;
        else if (row == 6) y_offset = 3.0;
        else if (row == 7) y_offset = 3.5;
        else if (row == 8) y_offset = 4.0;
        
        // 特殊处理某些位置
        if (flag_val == 5) x_offset = 1.0;  // 特殊位置
        if (flag_val == 3) x_offset = 1.0;  // 特殊位置
        if (flag_val == 2) x_offset = 1.0;  // 特殊位置
        if (flag_val == 1) x_offset = 1.0;  // 特殊位置
        if (flag_val == 0) x_offset = 1.0;  // 特殊位置
        if (flag_val == 52) y_offset = 1.0;  // 特殊位置
        if (flag_val == 42) y_offset = 1.0;  // 特殊位置
        if (flag_val == 32) y_offset = 1.0;  // 特殊位置
        if (flag_val == 22) y_offset = 1.0;  // 特殊位置
        if (flag_val == 12) y_offset = 1.0;  // 特殊位置
        
        // 设置目标位置
        pose.pose.position.x = init_x + x_offset;
        pose.pose.position.y = init_y + y_offset;
        pose.pose.position.z = init_z + ALTITUDE;
    }
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

        // 使用新的函数处理位置设置
        if(flag >= 0 && flag <= 99 && flag != 92 && flag != 93 && 
           ros::Time::now() - last_request > ros::Duration(1.0))
        {
            setTargetPosition(flag, pose, init_position_x_take_off, 
                            init_position_y_take_off, init_position_z_take_off);
            last_request = ros::Time::now();
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
