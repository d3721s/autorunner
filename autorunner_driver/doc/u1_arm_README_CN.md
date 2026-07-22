# u1_arm 驱动（5 关节达妙电机机械臂）

自研机械臂 **u1-arm** 的 ROS2 驱动，5 个关节全部使用**达妙(Damiao)电机**，底层直接用达妙 CAN 协议控制（无厂商臂控制器）。

**对外 ROS 接口完全镜像睿尔曼 `rm_driver`**：话题名一一对应（仅前缀 `rm_driver/` → `u1_arm/`），消息字段逐字一致。经比对，197 个话题与 rm_driver 完全 1:1（0 缺失 0 多余）。

- 与 `autorunner_driver`（松灵 PiperX）**同包共存**，是独立的可执行 target `u1_arm_driver_node`。
- 消息定义在 `autorunner_ros_interfaces` 包的 `msg/u1_arm/` 子目录。

## 1. 架构

```
u1_arm_driver_node (单进程, MultiThreadedExecutor 8线程)
├── 节点 u1_arm_driver        —— 全部 u1_arm/*_cmd 订阅 + u1_arm/*_result 发布
│   ├── 200Hz 控制回路: TrajectoryExecutor::tick → 位置速度帧下发
│   ├── CanBus(rx线程) → MotorManager 反馈缓存
│   └── Kinematics(KDL, l0→l5): MoveL/C/JP 的 FK/IK
└── 节点 u1_udp_publish_node  —— joint_states + u1_arm/udp_* 状态话题(周期 udp_cycle)
```

达妙协议要点（`include/u1_arm_driver/protocol/damiao_protocol.hpp`）：
- 控制帧 ID = 电机 CAN ID + 模式偏移（MIT +0x000 / 位置速度 +0x100 / 速度 +0x200 / 力位 +0x300）
- 使能 `FF FF FF FF FF FF FF FC` / 失能 `…FD` / 设零 `…FE` / 清错 `…FB`
- 位置速度帧：`float32 位置(rad) + float32 速度(rad/s)`，小端
- 反馈帧（ID=master_id）：`D0=ID|(ERR<<4)`，POS16/VEL12/T12 线性映射，D6/D7 温度
- 寄存器：广播 `0x7FF`，`0x33`读 / `0x55`写 / `0xAA`存 / `0xCC`读反馈
- 默认控制模式：**MIT 模式**（500Hz，`control_cycle_ms` 可配；每轴 `mit_kp`/`mit_kd` 需现场整定，Kd 必须 >0）；`*_canfd` 透传时运行时切到**位置速度模式**。详见 `doc.md`

## 2. 编译

```bash
cd ~/ros2_ws
colcon build --packages-select autorunner_ros_interfaces autorunner_driver
source install/setup.bash
```
依赖：`rclcpp` `ros2_socketcan` `kdl_parser` `orocos_kdl` `sensor_msgs` `geometry_msgs` `autorunner_ros_interfaces`。

## 3. 真机：USB2CAN → SocketCAN

达妙 USB2CAN 模块在 Linux 下需先落地为 SocketCAN 接口 `can0`（1 Mbps）：

```bash
# 若模块走 gs_usb (免驱), 插入后通常直接出现 can0:
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
# 若走 slcan (串口式), 用 can-utils:
sudo slcand -o -c -s8 /dev/ttyACM0 can0   # s8=1Mbps
sudo ip link set up can0
candump can0    # 抽查是否有帧
```
终端电阻：总线需 120Ω，测量总阻应约 60Ω（两端各一颗并联）。

## 4. 仿真调试（vcan0，无硬件）

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
# 启动 5 电机仿真器
python3 install/autorunner_driver/lib/autorunner_driver/fake_motors.py --iface vcan0 &
# 启动驱动
ros2 launch autorunner_driver u1_arm_driver.launch.py can_interface:=vcan0
```

## 5. 启动与自检

```bash
ros2 launch autorunner_driver u1_arm_driver.launch.py can_interface:=can0
ros2 topic hz /joint_states          # ≈200Hz
ros2 topic echo /u1_arm/udp_joint_en_flag   # 各关节使能状态

# MoveJ 到位 (5 关节, rad)
ros2 topic pub --once /u1_arm/movej_cmd autorunner_ros_interfaces/msg/U1Movej \
  "{joint: [0.3,0.5,0.5,0.3,0.2], speed: 40, block: true, trajectory_connect: 0, dof: 5}"
ros2 topic echo /u1_arm/movej_result        # data: true
# 急停(保持位置, 不失能) / 解除
ros2 topic pub --once /u1_arm/emergency_stop_cmd autorunner_ros_interfaces/msg/U1Stop "{state: true}"
ros2 topic pub --once /u1_arm/emergency_stop_cmd autorunner_ros_interfaces/msg/U1Stop "{state: false}"
```

## 6. 与 rm_driver 的接口对照

| rm_driver | u1_arm | 消息类型 |
|---|---|---|
| `rm_driver/movej_cmd` | `u1_arm/movej_cmd` | `U1Movej`(=rm `Movej`) |
| `rm_driver/movec_cmd` | `u1_arm/movec_cmd` | `U1Movec`(=rm `Movec`) |
| `rm_driver/emergency_stop_cmd` | `u1_arm/emergency_stop_cmd` | `U1Stop`(=rm `Stop`) |
| `rm_driver/udp_joint_error_code` | `u1_arm/udp_joint_error_code` | `U1Jointerrorcode` |
| `rm_driver/udp_joint_current` | `u1_arm/udp_joint_current` | `U1Jointcurrent` |
| `rm_driver/udp_joint_speed` | `u1_arm/udp_joint_speed` | `U1Jointspeed` |
| `rm_driver/udp_joint_temperature` | `u1_arm/udp_joint_temperature` | `U1Jointtemperature` |
| `rm_driver/udp_joint_voltage` | `u1_arm/udp_joint_voltage` | `U1Jointvoltage` |
| `joint_states` | `joint_states` | `sensor_msgs/JointState`(无前缀) |
| 其余全部 | `rm_driver/X` → `u1_arm/X` | 与 rm 同名同字段 |

> 8 个消息因 rosidl 拍平子目录与 autorunner 现有消息重名，加 `U1` 前缀；字段与 rm 逐字一致。QoS：cmd/result 用 `ParametersQoS()`，状态话题用 `QoS(10)`——与 rm 相同。

## 7. 电机标定（首次上电必做）

达妙电机的 CAN ID / Master ID / 控制模式 / 零点需按实物设定。用随包 CLI：

```bash
IFACE=can0
# 1) 逐一确认能通信 (读反馈)
python3 .../motor_setup.py --iface $IFACE refresh 1
# 2) 设定控制模式=位置速度(2) 并存 flash (须先失能)
python3 .../motor_setup.py --iface $IFACE disable 1
python3 .../motor_setup.py --iface $IFACE write 1 0x0A 2
python3 .../motor_setup.py --iface $IFACE save  1 0x0A
# 3) 关节移到机械零位后设零 (须失能)
python3 .../motor_setup.py --iface $IFACE zero 1
```
然后把每个电机的 CAN ID / Master ID / 量程填入 `config/u1_arm_driver.yaml`。

## 8. 参数（`config/u1_arm_driver.yaml`）

| 参数 | 说明 |
|---|---|
| `arm_dof` / `arm_joints` | 关节数(5) / 关节名 `[j1..j5]`（与 URDF 一致） |
| `can_interface` | SocketCAN 接口（真机 `can0`，仿真 `vcan0`） |
| `udp_cycle` | 控制/发布周期(ms)，5→200Hz |
| `auto_enable` | 启动即使能全部电机 |
| `estop_disable_motors` | 急停是否失能（默认 false=锁存位置，防跌落） |
| `base_link`/`tip_link` | KDL 链端点（`l0`/`l5`） |
| `motor_can_ids`/`motor_master_ids` | 每电机 CAN ID / 反馈 ID（数组，下标对齐 `arm_joints`） |
| `motor_p_max`/`v_max`/`t_max` | **定点映射量程，必须与电机内部设置一致** |
| `motor_direction`/`zero_offset` | 关节-电机方向 / 零位偏移 |
| `joint_lower`/`upper`/`vmax`/`amax` | 关节软限位 / 规划限速限加速 |

## 9. 功能矩阵

**已实现**（走达妙协议真实动作）：MoveJ（五次多项式同步插值）、MoveL/MoveC/MoveJ_P/MoveL_offset（KDL LMA IK，位置优先）、透传 movej_canfd/movep_canfd、move_stop/pause/continue、emergency_stop（锁位保持）、关节/笛卡尔示教 jog、set_joint_err_clear（重发使能）、get_current_arm_state/original_state、版本查询、工作/工具坐标系（软件注册表）、realtime_push、joint_states + udp_joint_{speed,temperature,current,voltage,en_flag,error_code,pose_euler} + udp_arm_position。

**打桩**（话题存在，回 `result=false` 或默认 `state=false`，硬件不存在）：六维力、夹爪、灵巧手、升降关节、扩展关节、Modbus/RS485、轨迹文件、在线编程/工程下发。

## 10. 注意事项 / 待现场确认

1. **量程 P/V/T_MAX** 与位置速度帧 float 布局须用达妙调试助手/官方协议手册核对，填入 yaml。
2. 关节↔电机型号/CAN ID 映射（CSV 有歧义）现场按实物填写。
3. 5 自由度**无法全姿态跟踪**：MoveL/MoveC 姿态为“尽力而为”（LMA 权重位置 1.0 / 姿态 0.1），目标不可达时 `result=false`。
4. 电压：达妙反馈帧无电压字段，`udp_joint_voltage` 初版恒 0（后续可用 `0x7FF` 低速轮询补）。
5. 电机内部 CAN ID 必须 ≤ 15（反馈帧 D0 低 4 位与状态位共享）。
```
