# Fast-LIO 自主无人机定位与航点飞行

基于 Fast-LIO 激光里程计、Livox MID360 激光雷达与 PX4 飞控的自主无人机定位与航点飞行系统。  
已在 **2024 年全国大学生电子设计竞赛** 中验证，得分 **100/120**。

## 硬件配置

| 组件 | 型号/规格 |
|------|-----------|
| 飞控 | CUAV V5+（PX4 固件 v1.13） |
| 激光雷达 | Livox MID360 |
| 视觉模块 | OpenMV 4 Plus |
| 机架 | 330mm 轴距 全碳板 |
| 电机 | 2207 6 inch |
| 电池 | 4S 9000mAh |
| 上位机 | NVIDIA Jetson / 树莓派（运行 ROS） |

![硬件图片](https://github.com/user-attachments/assets/b4e4fbb1-d6cc-46fa-8001-700f310332dc)

## 系统架构

```
MID360 LiDAR → livox_ros_driver2 → Fast-LIO (里程计)
                                        ↓
                              lio-to-mavros (坐标转换)
                                        ↓
下位机(CUAV V5+) ←→ uart_node ←→ offboard_node (控制)
     ↓
  电机/舵机
```

## 环境依赖

- **操作系统**：Ubuntu 20.04（推荐）或 Ubuntu 18.04
- **ROS 版本**：ROS Noetic（Ubuntu 20.04）或 ROS Melodic（Ubuntu 18.04）
- **PX4 固件**：v1.13
- **编译工具**：catkin

## 快速开始

### 1. 安装 Fast-LIO

```bash
# 克隆 Fast-LIO 仓库并按照官方文档编译
git clone https://github.com/hku-mars/FAST_LIO.git
```

### 2. 安装 Livox ROS Driver 2

```bash
# 克隆并编译 Livox ROS 驱动
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 3. 安装 MAVROS

```bash
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
# 安装 GeographicLib 数据集
sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh
```

### 4. 克隆本仓库并编译

```bash
cd ~/catkin_ws/src
git clone <your-repo-url> fastlio_to_mavros
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 5. 配置运行

编辑 `run_all.sh`，将 `ROS_IP` 修改为你的上位机 IP 地址：

```bash
export ROS_IP=192.168.xxx.xxx   # 改为你的 IP
```

赋予执行权限：

```bash
chmod +x run_all.sh
```

### 6. 一键启动

```bash
./run_all.sh
```

该脚本将依次启动：
1. `roscore` — ROS 主节点
2. `livox_ros_driver2` — Livox MID360 驱动
3. `fast_lio mapping_mid360` — Fast-LIO 建图与定位
4. `mavros px4` — PX4 飞控通信
5. `lio-to-mavros_node` — 里程计坐标转换到 PX4 机体坐标系
6. `offboard_node` — Offboard 控制指令
7. `uart_node` — 串口通信（上下位机）

## 节点说明

| 节点 | 源文件 | 功能 |
|------|--------|------|
| `lio-to-mavros_node` | `src/lio-to-mavros_node.cpp` | 将 Fast-LIO 里程计信息转换到 PX4 机体坐标系 |
| `uart_node` | `src/uart_node.cpp` | 下位机与上位机串口通信 |
| `offboard_node` | `src/offboard_node.cpp` | 无人机 Offboard 模式控制指令 |

## 目录结构

```
├── run_all.sh                # 一键启动脚本
├── src/                      # ROS 节点源码
│   ├── lio-to-mavros_node.cpp
│   ├── uart_node.cpp
│   └── offboard_node.cpp
├── down/                     # 下位机配置与源代码（按键、LCD 航线计算）
├── uper/                     # 上位机配置信息
├── openmv/                   # OpenMV 视觉神经网络代码（MicroPython）
├── openmv - 副本/             # OpenMV 代码副本
└── 3D maker/                 # 3D 打印文件（无人机适配件）
```

## 使用说明

### 下位机

`down/` 目录包含下位机固件配置与源代码，支持通过**按键和 LCD 屏幕**计算航线，然后将航点信息通过串口发送给上位机执行。

### 视觉模块

`openmv/` 目录包含基于 MicroPython 的机器视觉神经网络代码，运行于 OpenMV H7 开发板。

> **注意**：实际使用效果不佳，**谨慎使用**。推荐使用 OpenCV 替代方案。

### 3D 打印件

`3D maker/` 目录包含适配该无人机的 3D 打印模型文件，可根据实际需求修改。

## 致谢

感谢以下成员对项目的支持与开发：

- UEM Arklab — Yusiyuan
- UEM Storm — Li Xiaojia
- UEM Storm — Zhou Zhenquan
- UEM Storm — The Captain of Tomato

祝师弟师妹们勇创佳绩！

## 演示视频

[![演示视频](https://img.shields.io/badge/Bilibili-演示视频-00A1D6?logo=bilibili)](https://b23.tv/Q0KBtWE)

## License

本项目仅用于学习和研究目的。
