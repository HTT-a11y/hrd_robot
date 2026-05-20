# HRD 双臂 MoveIt + Gazebo 联合仿真

基于 MoveIt 2 + ros2_control + Gazebo Classic 的 HRD 双臂机器人联合仿真系统。

## 整体架构

```
┌─────────────┐  规划轨迹    ┌──────────────┐  FollowJointTrajectory  ┌──────────┐
│  MoveIt 2   │ ──────────→ │ ros2_control │ ──────────────────────→ │  Gazebo  │
│  (MoveGroup)│             │ (GazeboSystem│                        │  (物理)   │
│             │ ←────────── │   controller │ ←────────────────────── │          │
│  + RViz2    │  关节状态    │   _manager)  │   关节状态(Position)     │          │
└─────────────┘             └──────────────┘                        └──────────┘
```

- **MoveIt 2**：运动规划引擎，调用 Pick-IK 求解逆运动学，生成关节轨迹
- **ros2_control (GazeboSystem)**：运行在 Gazebo 内部，接收轨迹指令并驱动仿真关节
- **Gazebo Classic**：物理仿真环境，提供重力、碰撞、关节动力学模拟
- **RViz2**：可视化规划结果、碰撞体、机器人状态

## 控制闭环

```
MoveIt 规划轨迹
    → /left_arm_controller/follow_joint_trajectory (action)
    → joint_trajectory_controller 执行插补
    → GazeboSystem 写入关节位置 (position command)
    → Gazebo 物理引擎仿真一步
    → 关节状态回读 (position + velocity state)
    → joint_state_broadcaster 发布 /joint_states
    → robot_state_publisher 发布 /tf
    → MoveIt 更新机器人当前状态
```

## 机器人模型

- **双臂**：左右各 7 自由度机械臂 (link_1 ~ link_7)
- **双手**：左右各 5 指灵巧手 (拇指 2DOF, 其余四指各 1DOF)
- **底盘**：4 个麦克纳姆轮 (仿真中固定)
- 共 26 个被控关节，全部使用**位置控制**接口

## 规划组

| 规划组 | 关节数 | 控制器 | 用途 |
|--------|--------|--------|------|
| `left_arm` | 7 | `left_arm_controller` | 左臂运动规划 |
| `right_arm` | 7 | `right_arm_controller` | 右臂运动规划 |
| `left_hand` | 6 | `left_hand_controller` | 左手抓取 |
| `right_hand` | 6 | `right_hand_controller` | 右手抓取 |
| `dual_arms` | 14 | 无独立控制器 | 双臂组合规划 |

## 依赖

```bash
# 系统依赖（ROS 2 Humble）
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-tf2-ros

# IK 求解器
sudo apt install -y ros-humble-pick-ik
```

## 编译

```bash
cd ~/HRD_ws/hrd_project/hrd_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 快速启动

### 1. 启动联合仿真（终端1）

```bash
ros2 launch hrd_moveit_config gazebo_moveit.launch.py
```

这会同时启动 Gazebo、MoveGroup、RViz，并按时间顺序自动加载所有组件。

#### 启动时序

| 延迟 | 事件 |
|------|------|
| t=0s | 启动 Gazebo (空白世界)、robot_state_publisher |
| t=10s | 在 Gazebo 中生成 HRD 机器人模型 (z=0.35m) |
| t=12s | 在 Gazebo 中生成红色目标圆柱体 (0.05, -0.70, 0.8) |
| t=15s | 加载并启动 ros2_control 控制器 |
| t=20s | 在 MoveIt 规划场景中发布目标碰撞体 |

MoveGroup 和 RViz 在 t=0s 即启动，之后随其他组件就绪自动建立连接。

#### 自定义参数

```bash
# 修改机器人初始高度
ros2 launch hrd_moveit_config gazebo_moveit.launch.py spawn_z:=0.5

# 指定自定义 RViz 配置
ros2 launch hrd_moveit_config gazebo_moveit.launch.py rviz_config:=/path/to/custom.rviz
```

### 2. 运行双臂抓取 Demo（终端2）

```bash
ros2 launch hrd_moveit_config run_cpp_demo.launch.py
```

Demo 执行流程：
1. 读取目标圆柱体位姿
2. 通过 tf2 计算左右手动态抓取位姿（模板变换矩阵）
3. Layer1：纯位置可达性检查 (2s 超时)
4. Layer2：严格姿态可达性检查 (0.1 rad 容差)
5. 左臂先执行预抓取→抓取，右臂后执行

### 3. 查看 TCP 实时位姿

```bash
ros2 run hrd_moveit_config print_tcp_pose.py --stream
```

## 关键配置文件

| 文件 | 作用 |
|------|------|
| `config/HRD .urdf.xacro` | 顶层 xacro，根据 use_fake_hardware 切换真假硬件 |
| `config/HRD .ros2_control.xacro` | ros2_control 硬件接口定义 (GazeboSystem, 26个关节) |
| `config/ros2_controllers.yaml` | 控制器配置 (4个 JointTrajectoryController + 1个 JointStateBroadcaster) |
| `config/moveit_controllers.yaml` | MoveIt 端控制器映射 (FollowJointTrajectory action) |
| `config/HRD .srdf` | 规划组定义、碰撞矩阵、默认姿态 |
| `config/kinematics.yaml` | Pick-IK 求解器配置 (global mode) |
| `config/joint_limits.yaml` | 关节限位与速度/加速度缩放因子 |
| `config/initial_positions.yaml` | 各关节初始位置 (均为 0.0) |

## 核心文件

| 文件 | 作用 |
|------|------|
| `launch/gazebo_moveit.launch.py` | 联合仿真主启动文件 |
| `launch/spawn_controllers.launch.py` | ros2_control 控制器加载 |
| `src/demo_dual_arm_reach.cpp` | C++ 双臂抓取主程序 |
| `scripts/spawn_target_object.py` | MoveIt 规划场景碰撞体发布 |
| `scripts/print_tcp_pose.py` | TCP 位姿实时打印工具 |

## ros2_control 架构说明

联合仿真**不需要**单独的 `ros2_control_node`。`gazebo_ros2_control` 插件在 Gazebo 内部完成了：

- 硬件接口管理 (`GazeboSystem` 插件)
- 内置 `controller_manager` 实例
- 从 `ros2_controllers.yaml` 加载并派生控制器

`spawn_controllers.launch.py` 仅通过 `controller_manager` 服务的 `load_controller` / `switch_controller` 调用来激活已定义的控制器。

## Gazebo 模式 vs 纯 MoveIt 模式

| | Gazebo 联合仿真 | 纯 MoveIt (FakeSystem) |
|---|---|---|
| URDF 参数 | `use_fake_hardware:=false` | `use_fake_hardware:=true` |
| 硬件接口 | `gazebo_ros2_control/GazeboSystem` | `mock_components/GenericSystem` |
| 物理仿真 | 有 (重力/碰撞) | 无 (瞬时执行) |
| 启动命令 | `gazebo_moveit.launch.py` | `demo.launch.py` |
| 适用场景 | 验证完整控制链路 | 快速测试规划算法 |

## 常用 topic 与 action

```bash
# 查看规划组 action
ros2 action list | grep follow_joint_trajectory

# 查看当前关节状态
ros2 topic echo /joint_states

# 查看控制器状态
ros2 control list_controllers

# 查看 MoveGroup 规划场景
ros2 topic echo /monitored_planning_scene
```

## 故障排查

**Gazebo 启动后机器人加载失败**
检查 `GAZEBO_MODEL_PATH` 是否正确设置（launch 文件已自动处理），确认 `turn_on_hrd_robot` 包已编译。

**控制器未加载**
等待 15 秒延迟后，控制器才会被派生。如仍失败，检查 `spawn_controllers.launch.py` 的输出日志。

**MoveGroup 无法规划**
确认 IK 求解器 `pick_ik` 已安装。查看 RViz 中 Planning Scene 是否正确显示机器人模型。

**关节状态不更新**
确认 `joint_state_broadcaster` 已激活。使用 `ros2 control list_controllers` 检查各控制器是否处于 `active` 状态。

**仿真时间不同步**
所有节点均使用 `use_sim_time:=true`，确认 `/clock` topic 正在发布 (由 Gazebo 自动发布)。
