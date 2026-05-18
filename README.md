# HRD Dual-Arm Grasping Demo

基于 MoveIt 2 + Pick-IK 的双臂协同抓取演示，支持 Gazebo 仿真。

## 快速启动

```bash
# 终端1：启动 Gazebo + MoveGroup + RViz
ros2 launch hrd_moveit_config gazebo_moveit.launch.py

# 终端2：运行抓取 Demo
ros2 launch hrd_moveit_config run_cpp_demo.launch.py

# 查看 TCP 实时位姿
ros2 run hrd_moveit_config print_tcp_pose.py --stream
```

## 核心文件

| 文件 | 作用 |
|------|------|
| `src/demo_dual_arm_reach.cpp` | C++ 抓取主程序 |
| `config/kinematics.yaml` | Pick-IK 求解器配置 (global mode) |
| `launch/gazebo_moveit.launch.py` | Gazebo + MoveGroup 联合启动 |
| `launch/run_cpp_demo.launch.py` | C++ Demo 启动（注入 kinematics 参数） |

## 算法流程

```
圆柱位姿
  ↓
  1. calculate_dynamic_grasp_poses() — tf2 模板相对位姿解算
  2. Layer1: 纯位置可达性 (2s/次)
  3. Layer2: 严格姿态可达性 (0.1 rad)
  ↓
找到最优角度 → 左臂先执行 → 右臂后执行
```

## 位姿解算

使用 RViz 示教模板 + tf2 矩阵乘法：

- **左臂**：相对圆柱偏移 `(+0.064, -0.115, +0.098)`，四元数从 RViz 实测
- **右臂**：相对圆柱偏移 `(-0.121, -0.088, -0.023)`，四元数从 RViz 实测
- 预抓取位 = 抓取位 Z+10cm

圆柱坐标在 `main()` 中修改即可自动泛化。TCP link = 手心位置。

## 关键配置

- **IK 求解器**：`pick_ik/PickIkPlugin`，`mode: global`
- **运动学链**：arm chain tip = `tcp_link`（手心虚拟连杆）
- **碰撞**：默认开启（手指 STL 网格），圆柱碰撞体已注释

## 依赖

- ROS 2 Humble
- MoveIt 2
- Pick-IK (`pick_ik`)
- Gazebo + gazebo_ros2_control
