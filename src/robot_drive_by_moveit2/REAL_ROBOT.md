# 真机部署与仿真打通指南（5-DOF 机械臂 + ZP25S 舵机 + 香橙派）

> 本文档面向**真实机械臂**：从 MoveIt2 仿真，到香橙派上让真机动起来的完整记录。
> 仿真入门请见 [README.md](./README.md)。

## 目录

1. [项目与包结构](#一项目与包结构)
2. [硬件清单](#二硬件清单)
3. [机械臂模型（URDF）关键参数](#三机械臂模型urdf关键参数)
4. [运行形态总览：仿真 vs 真机](#四运行形态总览仿真-vs-真机)
5. [仿真模式（无需硬件）](#五仿真模式无需硬件)
6. [真机环境搭建（香橙派）](#六真机环境搭建香橙派)
7. [ZP25S 总线舵机协议](#七zp25s-总线舵机协议)
8. [真机校准数据表](#八真机校准数据表)
9. [真机打通：读方向（路线 A）](#九真机打通读方向路线-a)
10. [真机打通：执行方向（路线 B / ros2_control 插件）](#十真机打通执行方向路线-b--ros2_control-插件)
11. [安全规范](#十一安全规范)
12. [常见问题排查](#十二常见问题排查)
13. [路线图](#十三路线图)

---

## 一、项目与包结构

ROS2 Humble + MoveIt2 工作区，6 个功能包：

```
src/robot_drive_by_moveit2/
├── my_robot_description      # URDF/xacro 模型（5-DOF 臂 + 夹爪），含仿真与真机 ros2_control
├── my_robot_moveit_config    # MoveIt2 配置（SRDF、控制器、运动学、限位）
├── my_robot_bringup          # 一键启动：仿真 launch + 真机 launch + 控制器 yaml
├── my_robot_commander_cpp    # C++ 控制节点（test_moveit / commander）+ Python 真机脚本
├── my_robot_interfaces       # 自定义消息（PoseCommand）
└── my_robot_hardware         # C++ ros2_control 硬件插件（ZP25S 真机驱动）
```

工作区目录为 `~/moveit2_ws`（香橙派上）或任意 colcon 工作区。

---

## 二、硬件清单

| 部件 | 型号/说明 |
|---|---|
| 主控 | 香橙派 Orange Pi 4 Pro（全志 A733 8 核，建议 8GB+），运行 Ubuntu 22.04 (arm64) |
| 关节舵机 | 众灵科技 ZP25S 总线舵机 ×5（TTL 串口，270°/360° 视安装） |
| 夹爪 | 舵机驱动两指夹爪（夹爪舵机暂未分配总线 ID） |
| 串口 | USB-TTL 转串口模块（`/dev/ttyACM0`，115200） |
| （规划中）视觉 | 奥比中光 Astra Pro 深度相机（手眼标定/视觉引导） |
| 开发机 | PC（Windows/WSL 或 Ubuntu），仅用于开发/调试，不参与真机运行 |

---

## 三、机械臂模型（URDF）关键参数

### 3.1 运动链

5-DOF + 夹爪，经典"偏航 + 三俯仰 + 一滚转"布局：

```
base_link
 └ joint1   偏航 Z  continuous 360°
    └ shoulder_link（立柱 29.3mm）
       └ joint2   俯仰 Y  revolute  -45°~+90°
          └ arm_link（上臂 118mm）
             └ joint3   俯仰 Y  revolute  ±90°
                └ elbow_link（前臂 92mm）
                   └ joint4   俯仰 Y  revolute  ±90°
                      └ forearm_link（腕 31mm）
                         └ joint5   滚转 Z  continuous 360°
                            └ wrist_link → hand_link → tool_link（距 joint5 共 130mm）
                               └ gripper（prismatic 夹爪，mimic 同步）
```

### 3.2 连杆长度（实测，mm → URDF 中为米）

| 段 | 长度 |
|---|---|
| joint1 → joint2（肩立柱） | 29.3 mm |
| joint2 → joint3（上臂） | 118 mm |
| joint3 → joint4（前臂） | 92 mm |
| joint4 → joint5（腕） | 31 mm |
| joint5 → tool_link（到夹爪） | 130 mm |
| 基座离底盘顶 | 261 mm |

> 若重新组装或换结构，只需改 `my_robot_description/urdf/arm.xacro` 中的 `<origin xyz>`。

---

## 四、运行形态总览：仿真 vs 真机

| | 仿真（mock） | 真机（ZP25S） |
|---|---|---|
| URDF | `my_robot.urdf.xacro` | `my_robot.urdf.real.xacro` |
| ros2_control | `mock_components/GenericSystem` | `my_robot_hardware/Zp25sSystem` |
| launch | `my_robot.launch.xml` | `real_arm.launch.xml` |
| 控制器 yaml | `controller_manager.yaml` | `real_controllers.yaml` |
| 需要硬件 | 无 | 香橙派 + 串口 + 舵机 |

两套完全隔离：仿真文件零改动，真机通过 real 版文件接入。

---

## 五、仿真模式（无需硬件）

```bash
# 构建
colcon build --packages-select my_robot_description my_robot_moveit_config \
             my_robot_bringup my_robot_commander_cpp
source install/setup.bash

# 1) 完整 MoveIt2 系统（RViz 需图形界面）
ros2 launch my_robot_bringup my_robot.launch.xml

# 2) 运动示例（另开终端）
ros2 run my_robot_commander_cpp test_moveit

# 3) 话题指令指挥官
ros2 run my_robot_commander_cpp commander

# 4) 发送指令测试
ros2 topic pub /joint_command std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, -0.5, 0.0, 0.5]}"
ros2 topic pub /open_gripper std_msgs/msg/Bool "{data: true}"
```

> 5-DOF 臂无法到达任意 6D 位姿，`/pose_command` 只能发工作空间内可达的目标；
> 关节空间指令与命名目标（`home`）总是可达。

---

## 六、真机环境搭建（香橙派）

### 6.1 系统

- 系统镜像：**Ubuntu 22.04 (arm64)**（香橙派官网下载 Orange Pi 4 Pro 对应镜像）
- 验证：`uname -m` 输出 `aarch64`

### 6.2 ROS2 Humble + MoveIt2

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales && sudo locale-gen en_US en_US.UTF-8

sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update

sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update

sudo apt install -y ros-humble-moveit \
  ros-humble-controller-manager ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-joint-trajectory-controller ros-humble-moveit-simple-controller-manager
```

### 6.3 工程代码

```bash
mkdir -p ~/moveit2_ws/src && cd ~/moveit2_ws
git clone <你的仓库> src/robot_drive_by_moveit2
# 或从开发机拷贝源码（不要拷 build/install/log）

cd ~/moveit2_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/moveit2_ws/install/setup.bash" >> ~/.bashrc
```

### 6.4 串口权限

```bash
sudo usermod -aG dialout $USER   # 重新登录生效
ls -l /dev/ttyACM0
```

### 6.5 先验证仿真在香橙派上跑通

```bash
ros2 launch my_robot_bringup my_robot.launch.xml   # RViz 需要接 HDMI 显示器
```

---

## 七、ZP25S 总线舵机协议

ZP25S 为 ASCII 串口总线舵机（TTL，115200，8N1）：

| 功能 | 帧格式 | 示例 |
|---|---|---|
| 位置指令 | `#<ID>P<4位>T<4位>!` | `#002P1500T0500!` |
| 位置/方向模式 | `#<ID>PMOD1!` | 270° 模式 |
| 读当前位置 | `#<ID>PRAD!` | 返回 `#002P1500!` |
| 释放力矩 | `#<ID>PULK!` | 手动掰动关节用 |
| 修改 ID | 按厂家手册（建议单接舵机后用广播） | |

- ID 为 3 位数字（0~254），补零如 `001`；广播 ID `255`
- `P` 值典型范围 500~2500，**P=1500 为舵机"零位"**
- 舵机上电自动回到零位

> ⚠️ 舵机接线必须 TTL 电平（不可直接接 RS232）。供电 5~8.4V，建议 7.4V 独立供电，勿从香橙派取电。

---

## 八、真机校准数据表

校准前提：**重新组装时让每个舵机回零（P=1500）后，机械臂竖直向上**，使模型零位与真机零位对齐。

| 关节 | 舵机 ID | 零位 P | URDF 限位 | 实测方向 |
|---|---|---|---|---|
| joint1 | 001 | 1500 | continuous 360° | +1 |
| joint2 | 002 | 1500 | -45° ~ +90° | +1 |
| joint3 | 003 | 1500 | ±90° | +1 |
| joint4 | 004 | 1500 | ±90° | +1 |
| joint5 | 005 | 1500 | continuous 360° | +1 |
| gripper | 未分配 | — | prismatic | — |

**P ↔ 角度换算**（实测：P 从 1500 到 2400，转 ≈120°）：

```
1 P-step = 120 / 900 = 0.1333 度/步
URDF角(deg) = (P − 1500) × 0.1333 × direction
P = 1500 + URDF角(deg) / 0.1333 × direction     （direction = +1 或 -1）
```

> 若某个关节方向与模型相反：把对应配置的 `direction` 改为 -1，无需改机械结构。

---

## 九、真机打通：读方向（路线 A）

作用：轮询 5 个舵机 `PRAD` → 换算 URDF 弧度 → 发布 `/joint_states`，使 RViz 模型跟随真机（**现实 → 仿真**）。

```bash
# 终端 1：模型 + 只读桥（不带 RViz）
ros2 launch my_robot_bringup real_bridge.launch.xml

# 带 RViz（需要显示器）
ros2 launch my_robot_bringup real_bridge.launch.xml use_rviz:=true

# 也可以单独跑桥（先做方向演示扫动）
ros2 run my_robot_commander_cpp real_joint_state_bridge.py --demo
ros2 run my_robot_commander_cpp real_joint_state_bridge.py    # 正常只读轮询 ~10Hz
```

验证：
```bash
ros2 topic echo /joint_states    # 竖直时 joint1~5 ≈ 0；转动时数值跟随
```

涉及文件：`src/my_robot_commander_cpp/scripts/real_joint_state_bridge.py`
（关节 ID、方向在文件顶部 `JOINTS` 表配置）

> 说明：路线 A 不经过 MoveIt/ros2_control，专用于验证映射与方向。

---

## 十、真机打通：执行方向（路线 B / ros2_control 插件）

作用：MoveIt 规划的轨迹经 `arm_controller` 写入真机硬件插件，插件将弧度换算为 P 值并经串口驱动舵机（**仿真 → 现实**）。

### 10.1 架构

```
RViz/MoveIt (move_group)
   │  规划 joint1~5 轨迹
   ▼
arm_controller (joint_trajectory_controller, 50Hz)
   │  写 position 命令
   ▼
my_robot_hardware / Zp25sSystem（替换 mock）
   │  write(): 弧度 → P → 串口 #IDPxxxxTxxxx!
   │  read():  #IDPRAD! → 弧度 → 状态接口
   ▼
joint_state_broadcaster → /joint_states → RViz 跟随真机
```

### 10.2 使用流程（首次务必按顺序）

```bash
# 0) 编译新包
colcon build --packages-select my_robot_hardware my_robot_description \
             my_robot_moveit_config my_robot_bringup my_robot_commander_cpp
source install/setup.bash

# 1) 第一步：启动真机系统，先验证读回
ros2 launch my_robot_bringup real_arm.launch.xml
ros2 topic echo /joint_states     # 竖直时 5 关节 ≈ 0

# 2) 第二步：首次小范围试动（不经 MoveIt，人在急停旁）
ros2 run my_robot_commander_cpp arm_test_motion.py   # joint2 缓慢 +20° 再回 0

# 3) 第三步：MoveIt 闭环
ros2 run my_robot_commander_cpp test_moveit
ros2 run my_robot_commander_cpp commander
```

### 10.3 涉及文件

| 文件 | 作用 |
|---|---|
| `src/my_robot_hardware/` | C++ 插件：`Zp25sSystem`（read/write/串口 termios）|
| `src/my_robot_description/urdf/my_robot.urdf.real.xacro` | 真机 URDF |
| `src/my_robot_description/urdf/my_robot.ros2_control.real.xacro` | 真机 ros2_control 块（舵机 ID/方向参数）|
| `src/my_robot_bringup/config/real_controllers.yaml` | 真机控制器（50Hz，无 gripper）|
| `src/my_robot_bringup/launch/real_arm.launch.xml` | 真机一键启动 |
| `src/my_robot_commander_cpp/scripts/arm_test_motion.py` | 首次安全试动脚本 |

### 10.4 调整舵机 ID / 方向

改 `my_robot.ros2_control.real.xacro` 硬件参数即可，无需改 C++：

```xml
<param name="joint2_servo_id">2</param>
<param name="joint2_direction">1</param>     <!-- 反了就改成 -1 -->
<param name="move_time_ms">500</param>        <!-- 运动时长，慢速优先 -->
```

---

## 十一、安全规范

1. **限幅兜底**：插件对 P 值限幅 500~2500；joint2/3/4 的 URDF `<limit>` 同时约束控制器。
2. **慢速优先**：首次试动用 `move_time_ms` 较大值（500ms），验证后再调小。
3. **首次试动**：用 `arm_test_motion.py`（不经 MoveIt 规划），人在断电开关/急停旁。
4. **方向未验证前**：不要运行大范围规划；发现方向反了立即停，改 `direction` 后重编。
5. **供电**：机械臂舵机独立供电（7.4V），与香橙派电源分离；派需散热。
6. **夹爪无总线 ID**：当前真机控制器不含夹爪，避免误动作；待夹爪舵机设 ID 后再接入。

---

## 十二、常见问题排查

**Q1：launch 报 `mapping values are not allowed here`（URDF 相关）**
> robot_description 参数按 YAML 解析，URDF/xacro 的注释里**不能出现"半角冒号+空格"**（如 `note: xxx`）。
> 修复：把 xacro/urdf 注释里的 `: ` 改成 `- ` 或全角冒号。本仓库文件已遵守。

**Q2：RViz 启动崩溃 `could not connect to display`**
> 香橙派无显示器/未设置 DISPLAY。方案：接 HDMI 本地跑，或从 PC 远程 RViz（需 DDS 多机配置），或无头运行仅用命令行/话题验证。

**Q3：move_group 日志出现 octomap 传感器加载错误**
> 来自 `sensors_3d.yaml` 中 kinect 传感器配置，无相机时无实际影响；接入 Astra Pro 后再启用。

**Q4：找不到插件 `my_robot_hardware/Zp25sSystem`**
> `my_robot_hardware` 未编译或未 source。执行 `colcon build --packages-select my_robot_hardware` 并 `source install/setup.bash`。

**Q5：某关节方向与模型相反**
> 改 real xacro 中该关节 `direction` 为 `-1`，重新 build + source。

**Q6：P 值斜率不准**
> 重新实测：发 `P=1500` 与 `P=2400`，量输出轴实际角度，替换代码/配置中的 `0.1333`。

**Q7：舵机不响应 / 读取超时**
> 检查串口设备名、`dialout` 权限、TTL 电平接线、供电；用厂家工具先单独验证舵机。

---

## 十三、路线图

- [x] 5-DOF + 夹爪 URDF 模型（实测尺寸/限位）
- [x] MoveIt2 仿真完整链路（规划/执行/commander）
- [x] 香橙派 4 Pro + Ubuntu22.04 + ROS2 Humble + MoveIt2 部署
- [x] ZP25S 串口协议打通（读写、零位、斜率校准）
- [x] 路线 A：真机 → `/joint_states` → RViz 跟随
- [x] 路线 B：ros2_control 硬件插件，MoveIt → 真机执行
- [ ] 夹爪舵机分配总线 ID 并接入 `gripper_controller`
- [ ] Astra Pro 深度相机接入 + 手眼标定（eye-in-hand / eye-to-hand）
- [ ] 视觉引导抓取闭环（相机 → 目标位姿 → MoveIt → 真机）

---

*维护提示：本文档对应的代码变更请保持与本仓库一致；给 URDF/xacro 加注释时避免半角冒号。*
