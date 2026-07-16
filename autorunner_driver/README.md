# autorunner_driver

基于 `ros2_socketcan` 的 AgileX PiperX 机械臂 CAN 驱动 (ROS2 Humble)。

对照《机械臂对外通信协议》(见 `doc/`),按通信语义把接口分为 **Topic / Service / Action**:
请求-响应类用 Service, 长时间运动用 Action, 周期反馈与急停用 Topic。

## 启动

```bash
ros2 launch autorunner_driver autorunner_driver.launch.py can_interface:=can0
# 调试用虚拟 CAN:
#   sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
#   ros2 launch autorunner_driver autorunner_driver.launch.py can_interface:=vcan0
```

## Action (运动指令)

均带过程反馈, 支持取消 (取消时下发 0x150 急停)。完成判定基于 0x2A1 状态反馈:
byte4 运动状态 (到达/未到达) + byte1 机械臂状态 (0 正常, 非 0 表示无解/奇异/超限/碰撞等)。

| Action | 类型 | CAN | 说明 |
| --- | --- | --- | --- |
| `~/move_joint` | `control_msgs/action/FollowJointTrajectory` | 0x155~157 + 0x151(J) | 关节运动, 兼容 MoveIt/ros2_control。本驱动不插值, 取轨迹**末点** |
| `~/move_p` | `autorunner_ros_interfaces/action/MoveP` | 0x152~154 + 0x151(P) | 末端位姿点到点 |
| `~/move_l` | `autorunner_ros_interfaces/action/MoveL` | 0x152~154 + 0x151(L) | 末端直线 |
| `~/move_c` | `autorunner_ros_interfaces/action/MoveC` | 0x152~154+0x158 x3 + 0x151(C) | 圆弧, 起点=当前末端位姿 |

```bash
ros2 action send_goal /autorunner_driver/move_p autorunner_ros_interfaces/action/MoveP \
  "{target: {x: 0.3, y: 0.0, z: 0.2, rx: 0, ry: 0, rz: 0}, speed: 20}"
```

## Service (请求-响应)

带 CAN 应答的 service 下发后等待对应应答帧, 命中或超时 (`response_timeout_ms`, 默认 1s) 返回。

| Service | 类型 | CAN 下发 → 应答 |
| --- | --- | --- |
| `~/enable_joint` | `EnableJoint` | 0x471 → 0x476(0x71) |
| `~/set_joint_zero` | `SetJointZero` | 0x475 → 0x476(0x75, 含零点成功位) |
| `~/clear_joint_error` | `ClearJointError` | 0x475 → 0x476(0x75) |
| `~/set_joint_acc` | `SetJointAcc` | 0x475 → 0x476(0x75) |
| `~/query_joint_limit` | `QueryJointLimit` | 0x472(1) → 0x473 |
| `~/query_joint_max_acc` | `QueryJointMaxAcc` | 0x472(2) → 0x47C |
| `~/query_end_vel_acc` | `QueryEndVelAcc` | 0x477(1) → 0x478 |
| `~/query_collision_level` | `QueryCollisionLevel` | 0x477(2) → 0x47B |
| `~/set_joint_limit` | `SetJointLimit` | 0x474 (无应答, 下发即返回) |
| `~/set_end_vel_acc` | `SetEndVelAcc` | 0x479 (无应答, 下发即返回) |
| `~/set_collision_level` | `SetCollisionLevel` | 0x47A (verify=true 时查 0x477(2)→0x47B 回读校验) |
| `~/set_motion_ctrl` | `SetMotionCtrl` | 0x150 byte1/byte2 轨迹/拖动示教 |
| `~/emergency_stop` | `std_srvs/srv/Trigger` | 0x150 便捷急停 (与 topic 并存) |

```bash
ros2 service call /autorunner_driver/enable_joint \
  autorunner_ros_interfaces/srv/EnableJoint "{joint_num: 7, enable: true}"
```

## Topic

**订阅 (PC → 臂)** — 需最低延迟, 保持 topic:

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `~/stop_cmd` | `Stop` | 快速急停/恢复 (0x150 byte0) |
| `~/joint_mit_cmd` | `Jointmit` | MIT 控制 (0x15A~0x15F, 待实现) |

**发布 (臂 → PC)** — 周期反馈:

`/joint_states` (sensor_msgs), `~/arm_status`, `~/arm_position`, `~/joint_angle`,
`~/joint_speed`, `~/joint_current`, `~/joint_motor_pos`, `~/joint_voltage`,
`~/joint_temperature`, `~/joint_error_code`;
`publish_raw_frames:=true` 时额外发布 `~/raw_rx` (原始帧诊断)。

## 参数

见 `config/autorunner_driver.yaml`。关键项:
`can_interface`, `auto_enable`, `install_pos`, `default_speed_percent`,
`response_timeout_ms`, `motion_timeout_ms`, `min_move_time_ms` (运动起步观察窗口,
防止上次到达残留导致秒完成误判), `feedback_id_offset`/`control_id_offset` (主从偏移)。
