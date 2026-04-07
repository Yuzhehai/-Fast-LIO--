#!bin/bash
source /opt/ros/noetic/setup.bash  
source /root/catkin_ros/devel/setup.bash 
source /root/catkin_livox_ros_driver2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311 
export ROS_IP=192.168.43.48
roscore &
PID1=$!
echo "启动 roscore"
sleep 20

# 在后台启动第一个launch文件
roslaunch livox_ros_driver2 msg_MID360.launch &
PID2=$!
echo "启动 msg_MID360.launch "
sleep 5  # 给驱动足够时间初始化

# 在后台启动第二个launch文件
roslaunch fast_lio mapping_mid360.launch &
PID3=$!
echo "启动 mapping_mid360.launch"
sleep 5

cd
roslaunch mavros px4.launch &
echo "启动PX4驱动launch"
PID4=$!
sleep 5

cd
cd ~/catkin_ros
source devel/setup.bash
rosrun fastlio_to_mavros lio-to-mavros_node &
PID5=$!
echo "启动fast—lio转mavros节点"
sleep 10

cd
cd ~/catkin_ros
source devel/setup.bash
rosrun fastlio_to_mavros offboard_node  &
PID6=$!
echo "启动offboard控制节点and uart接收"
sleep 5

cd
cd ~/catkin_ros
source devel/setup.bash
rosrun uart_pkg uart_node   &
PID7=$!
echo "启动offboard控制节点and uart接收"
sleep 5

wait 
