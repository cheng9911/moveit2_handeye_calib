
````markdown
# MoveIt2 Hand-Eye Calibration (moveit2_handeye_calib)

基于 **MoveIt2** 的 **通用手眼标定 (Hand-Eye Calibration) 工具**，适配多种 ROS 2 机械臂平台。  
目前仓库已在 **Franka + ROS 2 Humble** 平台上完成实现与验证，其他机械臂只需保证 **话题接口一致**（末端位姿、相机 TF 等），即可快速接入使用。  

---

## ✨ 功能特性

- 支持 **eye-in-hand**（相机挂载在机械臂末端）和 **eye-to-hand**（相机固定在环境中）两种模式。  
- 基于 **MoveIt2 Calibration** 框架，集成 ChArUco / ArUco 标定板采集。  
- 通过 MoveIt2 与 RViz2 提供交互式 GUI，标定过程可视化。  
- 标定结果以 **TF 发布**，可直接用于机器人视觉任务。  
- 多平台支持：目前已验证 **Franka **，未来可扩展至 UR、xArm 等机械臂。  

---

## 🔗 参考项目与文档

- **MoveIt2 Hand-Eye Calibration 教程**  
  [moveit.picknik.ai 文档](https://moveit.picknik.ai/humble/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html)  

- **MoveIt Calibration (ROS2 port)**  
  [AndrejOrsula/moveit2_calibration](https://github.com/AndrejOrsula/moveit2_calibration)  

- **Franka ROS2**  
  [franka_ros2](https://github.com/frankaemika/franka_ros2)  

---

## ⚙️ 环境依赖

建议配置：  
- **操作系统**: Ubuntu 22.04  
- **ROS 版本**: ROS 2 Humble (推荐)  
- **依赖软件包**:
  - `moveit2`  
  - `moveit_calibration`  
  - `tf2_ros`, `geometry_msgs`, `sensor_msgs`  
  - `OpenCV` (用于 ArUco/ChArUco 检测)  
  - 对应机械臂的 ROS2 驱动（如 `franka_ros2`）
  - `realsense2_camera`（如果使用 RealSense 相机）  

安装依赖：  
```bash
sudo apt install ros-humble-realsense2-*
sudo apt install ros-humble-moveit2-* 
````

---

## 📂 系统架构 & 代码结构

```text
moveit2_handeye_calib/
├── Franka_ros2/               # Franka 官方 ROS 2 驱动与支持包
│   ├── franka_bringup/        # 启动相关节点与配置
│   ├── franka_description/    # Panda 机械臂的 URDF/SRDF 模型
│   ├── franka_example_controllers/ # 控制器示例
│   ├── franka_fr3_moveit_config/   # MoveIt2 配置文件
│   ├── franka_gazebo/         # Gazebo 仿真支持
│   ├── franka_gripper/        # 夹爪驱动
│   ├── franka_hardware/       # 硬件接口
│   ├── franka_msgs/           # 消息定义
│   ├── franka_robot_state_broadcaster/ # 机器人状态广播
│   └── libfranka/             # C++ SDK 库
│
├── moveit2_calibration/       # MoveIt2 手眼标定功能包
│   ├── moveit_calibration_demos/   # 标定流程示例
│   ├── moveit_calibration_gui/     # RViz 插件与交互式 GUI
│   └── moveit_calibration_plugins/ # ArUco/ChArUco 插件与算法实现
│
└── README.md                  # 项目说明文档
```
---
1. **Franka_ros2**  
   - 提供 **Franka Panda 在 ROS2 下的完整支持**，包括硬件接口、运动控制器、仿真环境以及 MoveIt2 配置文件。  
   - 负责机械臂和 ROS2 生态系统的集成，保证能接收并执行运动控制命令。  

2. **moveit2_calibration**  
   - MoveIt2 官方的 **手眼标定功能包**，包含标定插件、图形化界面和示例程序。  
   - 提供基于 **ArUco/ChArUco** 标定板的采集与求解功能。  
   - 与 Franka 的 MoveIt2 配置结合，即可完成 **eye-in-hand** 和 **eye-to-hand** 标定。  

3. **顶层集成**  
   - 本仓库通过将 **Franka_ros2** 与 **moveit2_calibration** 统一放置在同一工作区，实现 **Franka + MoveIt2 + Calibration** 的一体化方案。  
   - 未来支持扩展至 **其他机械臂平台**，只需保证末端位姿与相机 TF 话题对齐即可。

---


## 🚀 使用方法


### 1. 克隆仓库

```bash
cd ~/ros2_ws/src
git clone https://github.com/cheng9911/moveit2_handeye_calib.git
```

### 2. 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### 3. 启动相机

如果使用 Intel RealSense 相机，运行：
```
ros2 launch realsense2_camera rs_launch.py

```
此时会发布相机图像话题和相机 TF，用于标定板检测。

### 4.启动 Franka Panda + MoveIt2
请根据实际机械臂的 IP 地址修改 robot_ip 参数（例如 172.168.0.2）：
```
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.168.0.2
```
这将启动 Franka 驱动、状态广播器以及 MoveIt2 控制接口。在rviz中可以看到机械臂模型以及加载标定的算法。


启动后：

* MoveIt2 控制机械臂采集姿态
* 相机节点检测标定板
* 计算并输出手眼变换
* TF 发布结果，可在 RViz 中验证

### 4. 多平台使用

如果使用其他机械臂（如 UR、xArm）：

* 确保提供 **末端位姿** 话题以及 **MoveIt2 控制接口**
* 其他部分无需修改

---
## 📺 演示视频

查看项目手眼标定演示视频：
[![演示视频](https://img.youtube.com/vi/BV18FYqz9ECG/0.jpg)](https://www.bilibili.com/video/BV18FYqz9ECG)


## 📜 许可证

MIT License

---

## 🙏 致谢

* [MoveIt Calibration](https://github.com/ros-planning/moveit_calibration) 提供的标定框架
* [franka\_ros2](https://github.com/frankaemika/franka_ros2) 提供的 Franka 驱动
* 以及所有开源社区贡献者

---

💡 **欢迎贡献**：如果你在其他机械臂平台上完成了适配，欢迎提交 PR！


