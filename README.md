# ros_ar3_example_md

# AR3 Core Software
该存储库提供了用于使用 ros_control 和 MoveIt 控制 AR3 臂的软件。我希望为各位机械臂爱好者提供一个探索使用 ROS 控制机械臂的起点。基线实现旨在适应原始硬件和固件，包括用于通信的消息结构。展望未来，我将尽我所能继续保持它的可访问性。我计划探索一些我想到的有趣概念，用于手臂视觉、抓手和人机交互，如果可能，我将在这里分享我的项目。

* [Overview](#概述)
* [Installation](#安装)
* [Usage](#用法)



## Overview
+ **ar3_control**
    + 通过 MoveIt 用户界面控制手臂
    + 提供移动组界面的演示
    
+ **ar3_description**
    + 机械臂硬件描述、urdf等的说明
    
+ **ar3_hardware_interface**
    + 用于硬件驱动程序的 ROS 接口，建立在 ros_control 框架上
    + 管理关节偏移、限制和关节和执行器消息之间的转换
    
+ **ar3_hardware_drivers**
    + 处理与电机控制器的通信

+ **ar3_microcontrollers**
    + 用于电机控制器的固件，即:teensy4.1

+ **ar3_moveit_config**
    + 用于运动规划的 MoveIt 模块
    + 通过 Rviz 控制手臂

+ **ar3_gazebo**
    + Gazebo 上的模拟


## 安装
*在 Ubuntu 18.04 上进行有限测试。*


* [Ubuntu 18.04](#Ubuntu)

* [ROS Melodic and MoveIt]
### Ubuntu
* 搭建Ubuntu 18.04操作系统
* 在Ubuntu 18.04下安装 [ROS Melodic and MoveIt]，
具体安装步骤可参照如下：
①安装ros开发环境
wget http://fishros.com/install -O fishros && . fishros
sudo apt-get install curl && curl http://fishros.com/tools/install/ros-melodic | bash
最后使用Ctrl+Alt+T打开一个新的终端，输入roscore如果有打印即安装成功
②安装rosdep工具
curl http://fishros.com/tools/install/rosdepc | bash 

* 创建 ROS 工作区   ar3_ws
  
mkdir -p ~/ar3_ws/src && cd "$_"
  
* 将文件拷贝到到工作区 `src` 中：

  工作区目录应该是这样的：
  ```
  ar_ws
  +-- src
  |   +-- ar3_control
  |   +-- ar3_description
  |   +-- ...
  ```
* 构建工作区：
  ```
  catkin_make
  ```
* 获取工作区：
  ```
  source ~/ar3_ws/devel/setup.bash
  ```
  您可以将其添加到 .bashrc 中，以便在每次打开新终端时自动运行：
  ```
  echo "source ~/ar3_ws/devel/setup.bash" >> ~/.bashrc
  ```
* 如果您还没有启用串口访问，请启用：
  ```
  sudo addgroup $USER dialout
  ```
  您需要注销并重新登录才能使更改生效。
	MOVEIT
	1、安装moveit
	sudo apt-get install ros-melodic-moveit

	2、配置环境
	source /opt/ros/melodic/setup.bash

	3、安装资源文件：
	sudo apt-get install ros-melodic-moveit-resources

## 设置
* **Hardware interface硬件接口**  
  - 在`ar3_hardware_interface/config/hardware_driver.yaml`中设置串口和波特率
  

## 用法
您将始终需要运行两个模块：

1. **Arm module** -这可以用于现实世界或模拟手臂
     + 要控制真实世界的手臂，您需要运行 `ar3_hardware_interface` 模块
     + 对于虚拟手臂，您需要运行 `ar3_gazebo` 模块
     +任何一个模块都会为 MoveIt 加载必要的硬件描述

2. **MoveIt 模块** - `ar3_moveit_config` 模块提供 MoveIt 界面和 RViz GUI，`ar3_control` 模块提供 MoveIt 用户界面，用于以编程方式设置目标

下面描述了模块的各种用例和运行它们的说明：

-----

### RViz 中的 MoveIt 演示
如果你对MoveIt不熟悉，建议先从这个开始探索RViz中的MoveIt规划。这既不包含真实世界也不包含模拟手臂，而仅包含加载在 RViz 中用于可视化的模型。
*机器人描述、moveit 界面和 RViz 都将在单个演示中加载启动文件
  ```
  roslaunch ar3_moveit_config demo.launch
  ```

-----  

### 使用 RViz 中的 MoveIt 控制真实世界的手臂

* 启动`ar3_hardware_interface` 模块，它将加载配置和机器人描述
  ```
 	roslaunch ar3_hardware_interface ar3_hardware_bringup.launch
  硬件接口还将启动硬件驱动程序并初始化与teensy的通信。您在启动节点时使用 `use_existing_calibrations` 参数跳过联合编码器校准序列 
  
  
* 启动 MoveIt 和 RViz
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
您现在可以在 RViz 中进行规划并控制真实世界的手臂。关节命令和关节状态将通过硬件接口更新。

-----

###在 RViz 中使用 MoveIt 控制 Gazebo 中的模拟手臂

* 启动`ar3_gazebo`模块，它将启动Gazebo模拟器并加载机器人描述
  ```
  roslaunch ar3_gazebo ar3_gazebo_bringup.launch
  ```
* 启动 Moveit 和 RViz
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
您现在可以在 RViz 中进行规划并控制模拟手臂。

-----
### 带有移动组接口的控制臂

**建议首先使用模拟手臂运行此演示，以确保编程目标对您的环境（和您的手臂）是安全的。不用说，这同样适用于编写自己的任务。**

这是从官方MoveIt教程修改的演示。与通过 RViz 手动设置目标相反，移动组界面允许我们以编程方式计划和执行移动，并提供更多功能，例如指定路径约束和规划笛卡尔移动。这也可以实现更复杂的任务，围绕障碍物等进行规划。

*  启动`ar3_gazebo`模块，它将启动Gazebo模拟器并加载机器人描述
*	为了控制真实世界的手臂，你只需要运行`ar3_hardware_interface`而不是如上所述的`ar3_gazebo`。*
  ```
  roslaunch ar3_gazebo ar3_gazebo_bringup.launch
  ```
* 启动 Moveit 和 RViz
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
* 启动移动组演示
  ```
  roslaunch ar3_control move_group_demo.launch
  ```
 按照命令行说明逐步完成演示。有关更多详细信息，请参阅“ar3_control”。

-----
