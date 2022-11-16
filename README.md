# ros_ar3_example_md

# AR3 핵심 소프트웨어
이 저장소는 ros_control 및 MoveIt을 사용하여 AR3 암을 제어하기 위한 소프트웨어를 제공합니다. 동료 로봇 팔 애호가들에게 ROS를 사용하여 로봇 팔을 제어할 수 있는 출발점을 제공하고 싶습니다. 기본 구현은 통신에 사용되는 메시지 구조를 포함하여 원래 하드웨어 및 펌웨어에 적응하도록 설계되었습니다. 앞으로는 접근성을 유지하기 위해 최선을 다하겠습니다. 팔 시야, 그리퍼, 인간과 컴퓨터의 상호 작용에 대해 떠오르는 몇 가지 흥미로운 개념을 탐구할 계획이며 가능하면 여기에서 내 프로젝트를 공유할 것입니다.

* [개요](#개요)
* [설치](#installation)
* [사용법](#사용법)

## 개요
+ **ar3_control**
    + MoveIt 사용자 인터페이스를 통한 팔 제어
    + 모바일 그룹 인터페이스 데모 제공
    
+ **ar3_description**
    + 로봇팔 하드웨어 설명, urdf 등 설명
    
+ **ar3_hardware_interface**
    + ros_control 프레임워크에 구축된 하드웨어 드라이버용 ROS 인터페이스
    + 조인트와 액추에이터 메시지 간의 조인트 오프셋, 제한 및 전환 관리
    
+ **ar3_hardware_drivers**
    + 모터 컨트롤러와의 통신 처리

+ **ar3_microcontrollers**
    + 모터 컨트롤러용 펌웨어, 예: teensy4.1

+ **ar3_moveit_config**
    + 모션 계획을 위한 MoveIt 모듈
    + Rviz를 통한 암 제어

+ **ar3_gazebo**
    + Gazebo에서 시뮬레이션


## 설치
*Ubuntu 18.04에서 제한된 테스트. *

* [우분투 18.04](#우분투)

* [ROS 멜로디와 무브잇]
### 우분투
* 빌드 우분투 18.04 운영 체제
* Ubuntu 18.04에서 [ROS Melodic 및 MoveIt] 설치,
특정 설치 단계는 다음을 참조할 수 있습니다.
①ros 개발 환경 설치
get http://fishros.com/install -O fishros && . fishros
sudo apt-get install curl && curl http://fishros.com/tools/install/ros-melodic | bash
마지막으로 Ctrl+Alt+T를 사용하여 새 터미널을 열고 roscore를 입력하고 인쇄가 있으면 설치가 성공한 것입니다.
② rosdep 툴 설치
curl http://fishros.com/tools/install/rosdepc | bash 

* ROS 워크스페이스 ar3_ws 생성
  
mkdir -p ~/ar3_ws/src && cd "$_"
  
* 파일을 작업공간 `src`에 복사합니다.

  작업 공간 디렉토리는 다음과 같아야 합니다.
  ```
  ar_ws
  +-- 소스
  | +-- ar3_control
  | +-- ar3_description
  | +-- ...
  ```
* 빌드 작업 공간:
  ```
  catkin_make
  ```
* 작업 공간 가져오기:
  ```
    source ~/ar3_ws/devel/setup.bash
  ```
  새 터미널을 열 때마다 자동으로 실행되도록 .bashrc에 이것을 추가할 수 있습니다.
  ```
echo "source ~/ar3_ws/devel/setup.bash" >> ~/.bashrc
  ```
* 직렬 액세스를 활성화하지 않은 경우 다음을 활성화하십시오.
  ```
  sudo addgroup $USER dialout
  ```
  변경 사항을 적용하려면 로그아웃했다가 다시 로그인해야 합니다.
이동
1. 무브잇 설치
sudo apt-get install ros-melodic-moveit

2. 구성 환경
source /opt/ros/melodic/setup.bash

3. 리소스 파일을 설치합니다.
sudo apt-get install ros-melodic-moveit-resources

## 설정
* **하드웨어 인터페이스하드웨어 인터페이스**
  - `ar3_hardware_interface/config/hardware_driver.yaml`에서 직렬 포트 및 전송 속도 설정
  

## 용법
항상 두 개의 모듈을 실행해야 합니다.

1. **팔 모듈** - 실제 또는 시뮬레이션된 팔에 사용할 수 있습니다.
     + 실제 팔을 제어하려면 `ar3_hardware_interface` 모듈을 실행해야 합니다.
     + 가상 팔의 경우 `ar3_gazebo` 모듈을 실행해야 합니다.
     +모든 모듈이 MoveIt에 필요한 하드웨어 설명을 로드합니다.

2. **MoveIt 모듈** - `ar3_moveit_config` 모듈은 MoveIt 인터페이스와 RViz GUI를 제공하고 `ar3_control` 모듈은 프로그래밍 방식으로 대상을 설정하기 위한 MoveIt UI를 제공합니다.

모듈의 다양한 사용 사례와 이를 실행하기 위한 지침은 아래에 설명되어 있습니다.

-----

### RViz의 MoveIt 데모
MoveIt에 익숙하지 않은 경우 여기부터 시작하여 RViz에서 MoveIt 계획을 탐색하는 것이 좋습니다. 여기에는 실제 세계나 시뮬레이션된 팔이 포함되지 않고 시각화를 위해 RViz에 로드된 모델만 포함됩니다.
*로봇 설명, moveit 인터페이스 및 RViz는 모두 단일 데모에서 시작 파일을 로드합니다.
  ```
  roslaunch ar3_moveit_config 데모.실행
  ```

-----

-----

### RViz에서 MoveIt을 사용하여 실제 팔 제어

* 구성 및 로봇 설명을 로드할 'ar3_hardware_interface' 모듈을 시작합니다.
  ```
 roslaunch ar3_hardware_interface ar3_hardware_bringup.launch
  하드웨어 인터페이스는 또한 하드웨어 드라이버를 시작하고 십대와의 통신을 초기화합니다. 노드를 시작할 때 `use_existing_calibrations` 매개변수로 조인트 인코더 보정 시퀀스를 건너뜁니다.
  
  
* MoveIt 및 RViz 시작
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
이제 RViz에서 실제 팔을 계획하고 제어할 수 있습니다. 관절 명령과 관절 상태는 하드웨어 인터페이스를 통해 업데이트됩니다.

-----

### RViz에서 MoveIt을 사용하여 Gazebo에서 시뮬레이션된 팔 제어

* Gazebo 시뮬레이터를 시작하고 로봇 설명을 로드하는 `ar3_gazebo` 모듈을 시작합니다.
  ```
  roslaunch ar3_gazebo ar3_gazebo_bringup.launch
  ```
* Moveit 및 RViz 시작
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
이제 RViz에서 시뮬레이션된 팔을 계획하고 제어할 수 있습니다.

-----

### 모바일 그룹 인터페이스가 있는 컨트롤 암

**프로그래밍 대상이 환경(및 팔)에 안전한지 확인하기 위해 먼저 더미 팔로 이 데모를 실행하는 것이 좋습니다. 말할 필요도 없이 자신의 작업을 작성하는 경우에도 마찬가지입니다. **

이것은 공식 MoveIt 튜토리얼에서 수정된 데모입니다. RViz를 통해 수동으로 목표를 설정하는 것과는 달리 이동 그룹 인터페이스를 사용하면 경로 제약 조건 지정 및 데카르트 이동 계획과 같은 추가 기능을 사용하여 프로그래밍 방식으로 이동을 계획하고 실행할 수 있습니다. 이것은 또한 더 복잡한 작업, 장애물 주변 계획 등을 가능하게 합니다.

* Gazebo 시뮬레이터를 시작하고 로봇 설명을 로드하는 `ar3_gazebo` 모듈을 시작합니다.
* 실제 암을 제어하려면 위와 같이 `ar3_gazebo` 대신 `ar3_hardware_interface`를 실행하면 됩니다. *
  ```
  roslaunch ar3_gazebo ar3_gazebo_bringup.launch
  ```
* Moveit 및 RViz 시작
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
* 모바일 그룹 데모 시작
  ```
  roslaunch ar3_control move_group_demo.launch
  ```
 데모를 통해 명령줄 지침을 단계별로 따르십시오. 자세한 내용은 'ar3_control'을 참조하십시오.

-----




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
