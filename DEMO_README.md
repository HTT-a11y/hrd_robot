# demo_dual_arm_reach.cpp 逻辑与流程文档

## 整体流程

```
HOME (关节空间)
  →  setJointValueTarget + plan + execute
  →  左右臂均回到预定义 HOME 位姿

计算预抓取目标
  →  calc_grasp() 基于圆柱位姿 + 接近角 + 距离参数
  →  生成 left_pre / right_pre (Z 轴抬升 safe_d 作为预抓取安全高度)

执行抓取
  →  plan_exec() 对每条臂独立规划 + 执行
  →  左臂先走 → 右臂后走
```

## 核心函数

### 1. `calc_grasp()` — 运动学解算

```cpp
calc_grasp(cyl_pose, left_ang, right_ang, grasp_d, safe_d, ...)
```

**输入：**
| 参数 | 当前值 | 含义 |
|------|--------|------|
| `cyl` | `(-0.10, -0.45, 0.80)` | 圆柱中心位姿 |
| `left_ang` | `0.0` (0°) | 左臂接近角，+X 方向 |
| `right_ang` | `M_PI` (180°) | 右臂接近角，-X 方向 |
| `grasp_d` | `0.10` | TCP 距圆柱中心距离 (10cm) |
| `safe_d` | `0.03` | 预抓取 Z 轴抬升 (3cm) |

**算法：**
1. 极坐标计算抓取位：`pos = cyl + (grasp_d * cos(θ), grasp_d * sin(θ), 0)`
2. 构建正交旋转矩阵：
   - `X_tcp = (0,0,1)` — 拇指向上，平行圆柱主轴
   - `Y_tcp = normalize(cyl - pos)` — 手心指向圆柱中心
   - `Z_tcp = X × Y` — 右手定则
3. 预抓取位 = 抓取位 + `safe_d` 沿世界 Z 轴

### 2. `plan_exec()` — 规划 + 执行

```cpp
plan_exec(log, move_group, pre_pose, arm_name)
```

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `GoalPositionTolerance` | `0.05` | 位置容差 5cm |
| `GoalOrientationTolerance` | `3.14` | 姿态容差 180° (纯位置约束) |
| `PlanningTime` | `15.0` | 单次规划超时 15s |
| `VelocityScalingFactor` | `0.5` | 速度缩放 50% |

## 工作空间约束 (已验证)

通过大面积扫描 (X=-0.35~+0.15, Y=-0.15~-0.55, Z=0.80) 得到：

| 臂 | 可达 X 范围 (Y=-0.45, Z=0.80) |
|----|-------------------------------|
| 右臂 | `[-0.35, -0.15]` |
| 左臂 | `[-0.15, -0.15]` (仅一点) |
| **交集** | **X = -0.15** |

**关键结论：** 双臂公共工作空间仅在 X=-0.15 附近很窄的一小片区域。所有目标必须落在此范围内。

## 配置与依赖

- **IK 求解器:** Pick-IK `mode: global`
- **运动学链:** `left_tcp_link` / `right_tcp_link` (手心虚拟连杆)
- **碰撞检测:** 规划时默认开启自碰检测；圆柱碰撞体已注释（可放开）
- **HOME 位姿:** 来自 SRDF 的 `home_left_arm` / `home_right_arm`

## 调试历史关键节点

1. **右臂 IK 长期失败** → 发现右臂可达空间在 X 负半轴 (-0.35~-0.15)，不是预期的正半轴
2. **左臂可达范围极窄** → 仅在 X=-0.15 附近一点可达
3. **Tcp_link vs link_7** → TCP 偏移 (12cm) 会改变手腕位置要求，影响 IK 求解
4. **rpy 修改** → 右臂 base_link 的 rpy 改动会导致 Pick-IK 完全不可收敛，已还原

## 运行

```bash
ros2 launch hrd_moveit_config gazebo_moveit.launch.py
ros2 launch hrd_moveit_config run_cpp_demo.launch.py
```
