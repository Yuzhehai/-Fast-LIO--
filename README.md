首先进行Fast-LIO项目建立并确保可以输出正确的topic
无人机硬件：
  1、2207 6inch电机
  2、4s 9000mah电池
  3、CUAV-V5+ 飞控 固件1.13
  4、视觉模块openmv4 plus
  5、330轴距全碳板机架

创建工作空间将src文件夹clone进去，使用catkin_make进行编译
在src文件夹下包含3个node节点
  1、lio to mavros：该节点是将Fast-LIO里程计信息转换到PX4机体坐标系下
  2、uart_node:该节点是下位机与上位机通过串口进行传输
  3、offboard_node:该节点是无人机控制指令节点
经验证可以顺利完成2024年电赛题目 100/120；

优化方向：offboard_node在标志位代码编写冗余可以使用单个子函数进行替换

后续更新：机架3D打印件；底层STM32代码；openmv数据集以及代码；CUAV PX4参数文件
