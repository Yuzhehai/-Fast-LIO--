首先进行Fast-LIO项目建立并确保可以输出正确的topic
无人机硬件：
  1、2207 6inch电机
  2、4s 9000mah电池
  3、CUAV-V5+ 飞控 固件1.13
  4、视觉模块openmv4 plus
  5、330轴距全碳板机架

图片
  ![51c031161437cb36e15dbde849eb0f08](https://github.com/user-attachments/assets/b4e4fbb1-d6cc-46fa-8001-700f310332dc)

创建工作空间将src文件夹clone进去，使用catkin_make进行编译
在src文件夹下包含3个node节点
  1、lio to mavros：该节点是将Fast-LIO里程计信息转换到PX4机体坐标系下
  2、uart_node:该节点是下位机与上位机通过串口进行传输
  3、offboard_node:该节点是无人机控制指令节点
  4、run_all.sh:该脚本可以打开所有代码 包括LIO、通信和控制节点，使用前将其赋予chmod +x 权限并设置开机启动
经验证可以顺利完成2024年电赛题目 100/120；
3D maker文件里包含无人机适配的打印文件可以根据实际需求进行更改

感谢 UEM Arklab Yusiyuan；UEM storm Lixiaojia；UEM strom ZhouZhenQuan 对项目的支持与开发；祝师弟师妹们勇创佳绩！

演示视频：【CUAV V5nano 融合Fast-LIO 自主无人机定位与航点测试飞行-哔哩哔哩】 https://b23.tv/Q0KBtWE
