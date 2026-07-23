# u1_arm 驱动接口大全（每个接口作用于哪个电机、做了什么）

本文逐一列出 `u1_arm_driver` 对外的全部 ROS 接口，并说明每个接口**作用于哪个电机**、在**达妙 CAN 协议层做了什么动作**。接口命名、字段、QoS 完全镜像 `rm_driver`（仅前缀 `rm_driver/` → `u1_arm/`）。

- 命令类：订阅 `u1_arm/<X>_cmd`，处理后发布 `u1_arm/<X>_result`。
- 状态类：周期发布 `joint_states` 与 `u1_arm/udp_*`。
- 所有关节角单位 **弧度**，速度百分比 **1~100**，`dof=5`。
- **底层控制默认 MIT 模式**（500Hz），仅 `*_canfd` 透传切到位置速度模式。

---

## 0. 关节 ↔ 电机映射（默认值，`config/u1_arm_driver.yaml` 可改）

| 关节 | 电机型号 | CAN ID | Master ID(反馈帧ID) | 软限位(rad) |
|---|---|---|---|---|
| j1 | DM-J4310 | 0x01 | 0x11 | [-0.1745, 1.7453] |
| j2 | DM-J4340 | 0x02 | 0x12 | [-0.061087, 1.6057] |
| j3 | DM-J4310 | 0x03 | 0x13 | [-0.087266, 3.1416] |
| j4 | DM3507 | 0x04 | 0x14 | [-0.22689, 3.8048] |
| j5 | DM3507 | 0x05 | 0x15 | [-1.5708, 1.5708] |

**关节角↔电机角换算**：`电机角 = direction × 关节角 + zero_offset`（`motor_direction` / `motor_zero_offset` 每轴可配）。

## 1. CAN 层操作原语（所有接口最终都归结为这几种帧）

**底层默认 MIT 模式**（力矩=Kp·位置误差+Kd·速度误差+前馈）。`<偏移>` 随当前模式变化：MIT=`0x000`、位置速度=`0x100`。

| 原语 | CAN 帧 | 说明 |
|---|---|---|
| MIT 控制 | ID=`0x000+CANID`, data=`P(16b)+V(12b)+Kp(12b)+Kd(12b)+T_ff(12b)` | **默认运动主力帧**；P=目标角, V=速度前馈(轨迹导数), Kp/Kd 每轴可配, T_ff=URDF/KDL 重力补偿(可关) |
| 位置速度控制 | ID=`0x100+CANID`, data=`float32 位置 + float32 速度`(小端) | 仅 `movej_canfd`/`movep_canfd` 透传时用 |
| 使能 | ID=`<偏移>+CANID`, data=`FF FF FF FF FF FF FF FC` | 在当前模式偏移使能 |
| 失能 | ID=`<偏移>+CANID`, data=`… FD` | 零力矩 |
| 清错 | ID=`<偏移>+CANID`, data=`… FB` | 清故障 |
| 设零 | ID=`<偏移>+CANID`, data=`… FE` | 存当前位置为零点 |
| 切模式 | ID=`0x7FF`, data=`CANID_L CANID_H 0x55 0x0A <mode> 00 00 00` | 写 CTRL_MODE（1=MIT / 2=位置速度）|
| 读反馈 | ID=`0x7FF`, data=`CANID_L CANID_H 0xCC 00 …` | 失能电机查询反馈 |
| 反馈帧(电机→驱动) | ID=`Master ID`, `D0=状态<<4\|ID低4位`, POS16/VEL12/T12, D6/D7温度 | 所有状态话题的数据源；被动式一发一收 |

**500Hz 控制回路（`control_cycle_ms`=2，可配）**：每 2ms 遍历 5 个电机 —— 已使能的按**当前模式**发控制帧（MIT 帧或位置速度帧，目标=当前 setpoint，同时引出反馈）；未使能的发 **0xCC** 查询反馈。MIT MoveJ 速度前馈=五次多项式解析导数，采样轨迹/jog 使用相邻 setpoint 差分；力矩前馈可由 URDF/KDL 计算重力补偿。经典 CAN 1Mbps 下 5 电机×2×500=5000fps，在 6000fps 预算内。**状态发布回路独立**，频率 `udp_cycle`（默认 5ms=200Hz）。

**运行时模式切换**：默认全程 MIT；仅当收到 `movej_canfd`/`movep_canfd` 时把 5 轴切到位置速度模式（失能→写 CTRL_MODE=2→在 0x100 偏移使能），下一次 MIT 类命令再切回。仅在模式真正变化时执行（约 15ms），故急停/停止**不切模式**（就地保持最快）。

**启动序列（`auto_enable=true`）**：对 5 个电机依次 `写 CTRL_MODE=MIT(1)` → `使能(FC @0x000)`；看门狗每 0.5s 对"有反馈但未使能"的电机重发。

---

## 2. 运动控制接口

| 接口(`u1_arm/…`) | 消息 | 作用电机 | 在电机上做了什么 |
|---|---|---|---|
| `movej_cmd` | U1Movej | **全部 5 轴** | 先确保 **MIT 模式**；驱动内五次多项式同步插值（`speed` 缩放 vmax/amax），控制环每 2ms(500Hz) 给 5 个电机发 **MIT 帧**（P=插值点、V=解析速度前馈、Kp/Kd 来自配置、T_ff=重力补偿）逼近目标；限位校验。`block:true` 等反馈到位再回 `result` |
| `movej_canfd_cmd` | Jointpos | **全部 5 轴** | **切到位置速度模式**，直接把 5 轴目标设为 setpoint，**不做规划**，控制环下发**位置速度帧**。即发即忘（**无 result**）|
| `movej_canfd_custom_cmd` | Jointposcustom | 全部 5 轴 | 同上（位置速度模式透传）|
| `movel_cmd` | Movel | **全部 5 轴** | 确保 MIT；末端笛卡尔直线每 2mm 插值，逐点 **KDL LMA IK**→采样轨迹，按 `control_cycle` 频率下发 **MIT 帧**。IK 失败/超限回 `false` |
| `movec_cmd` | U1Movec | **全部 5 轴** | 确保 MIT；三点圆弧（起点+`pose_mid`+`pose_end`，`loop` 圈数）→逐点 IK→采样下发 MIT 帧 |
| `movej_p_cmd` | Movejp | **全部 5 轴** | 目标位姿 → 单次 IK → 走 MoveJ（MIT）同步插值 |
| `movel_offset_cmd` | Moveloffset | **全部 5 轴** | 确保 MIT；当前位姿叠加 `pose` 偏移 → 直线 IK → 采样下发 MIT 帧 |
| `movep_canfd_cmd` | Cartepos | **全部 5 轴** | 位姿 IK → **位置速度模式**透传（无 result）|
| `movep_canfd_custom_cmd` | Carteposcustom | 全部 5 轴 | 同上 |

> 说明：除 `*_canfd` 透传走位置速度模式外，其余运动均走 **MIT 模式**。5 自由度笛卡尔类**位置优先、姿态尽力**（IK 权重位置 1.0 / 姿态 0.1），一次动作协同**全部 5 个电机**。MIT 增益 `mit_kp`/`mit_kd` 每轴可配，**需现场整定**（Kd 必须 >0 否则位置控制震荡）。

## 3. 停止 / 急停 / 暂停

| 接口 | 消息 | 作用电机 | 在电机上做了什么 |
|---|---|---|---|
| `move_stop_cmd` | Empty | 全部 5 轴 | 中止当前轨迹；冻结当前 setpoint 为目标、速度前馈→0，控制环继续发 MIT 帧 → 靠 Kp/Kd **平滑减速并原地保持**。不切模式、不改使能 |
| `emergency_stop_cmd` | U1Stop | 全部 5 轴 | `state:true`：中止+冻结当前位置(V=0)的 MIT 帧保持（默认**不失能**防跌落；`estop_disable_motors=true` 则发**失能帧 FD**）。`state:false`：以当前反馈为保持位解除。急停期间拒绝一切运动命令，**不切模式**(最快响应) |
| `pause_cmd` | Empty | 全部 5 轴 | 冻结轨迹进度（继续发当前 setpoint，电机停在原地）|
| `set_arm_continue_cmd` | Empty | 全部 5 轴 | 解除暂停，轨迹从冻结点继续 |

## 4. 示教（jog）

| 接口 | 消息 | 作用电机 | 在电机上做了什么 |
|---|---|---|---|
| `set_joint_teach_cmd` | Jointteach | **单轴**（`num`=1~5 → j1~j5）| 该轴恒速点动：控制环对**该电机**发位置速度帧、目标按 `direction`(1正/0负)、`speed` 递增，直到 `set_stop_teach` 或撞软限位自动停 |
| `set_pos_teach_cmd` | Posteach | **全部 5 轴** | 末端沿 `type`(0/1/2=x/y/z) 平移一小段（3cm×speed 比例）→ IK → 采样，故实际协同全部 5 轴 |
| `set_ort_teach_cmd` | Ortteach | **全部 5 轴** | 末端绕 `type`(rx/ry/rz) 旋转一小段（0.15rad×比例）→ IK → 采样 |
| `set_stop_teach_cmd` | Empty | 相关轴 | 停止上述任一 jog |

## 5. 关节维护

| 接口 | 消息 | 作用电机 | 在电机上做了什么 |
|---|---|---|---|
| `set_joint_err_clear_cmd` | Jointerrclear | **单轴**（`joint_num`=1~5）| 对**该电机**发**清错帧(FB)** 再发**使能帧(FC)**，用于清故障后恢复 |

---

## 6. 状态发布话题（数据全部来自电机反馈帧 / FK）

QoS=10，周期 `udp_cycle`（默认 5ms=200Hz）。

| 话题(`u1_arm/…` 或 `joint_states`) | 消息 | 数据来源与涉及电机 |
|---|---|---|
| `joint_states` | sensor_msgs/JointState | **全部 5 轴**：`position`=各电机反馈 POS(经 direction/zero_offset 换算)，`velocity`=反馈 VEL，`effort`=反馈扭矩 T |
| `udp_joint_speed` | U1Jointspeed | 5 轴反馈 VEL(rad/s) |
| `udp_joint_temperature` | U1Jointtemperature | 5 轴反馈 D7 线圈温度(℃) |
| `udp_joint_current` | U1Jointcurrent | 5 轴反馈扭矩 T 换算 |
| `udp_joint_voltage` | U1Jointvoltage | 恒 0（达妙反馈帧无电压字段）|
| `udp_joint_en_flag` | Jointenflag | 5 轴反馈状态位==1(使能) |
| `udp_joint_error_code` | U1Jointerrorcode | 5 轴反馈状态位（8~E 为故障码，0/1 视为无错）|
| `udp_arm_position` | geometry_msgs/Pose | 由 5 轴关节角 **FK** 得末端位姿（不直接读电机）|
| `udp_joint_pose_euler` | Jointposeeuler | FK 末端欧拉角+位置 |
| `udp_rm_err` | Rmerr | 看门狗检测到**反馈超时的电机编号**（1~5）|

> `udp_arm_position` / `udp_joint_pose_euler` 不与电机直接通信，是对 `joint_states`（来自 5 轴反馈）做正运动学得到。

## 7. 查询接口（不改变电机，只读/软件状态）

| 接口 | 结果消息 | 涉及电机 |
|---|---|---|
| `get_current_arm_state_cmd` | Armstate | **读** 5 轴反馈快照 + FK 位姿 |
| `get_current_arm_original_state_cmd` | Armoriginalstate | 同上（pose 为欧拉角数组）|
| `get_arm_software_version_cmd` | Armsoftversion | 无（静态字符串 "U1_ARM"）|
| `get_robot_info_cmd` | RobotInfo | 无（`arm_dof=5`）|
| `get_joint_software_version_cmd` | Jointversion | 无（占位版本号）|
| `get_tool_software_version_cmd` | Toolsoftwareversionv4 | 无（`state=false`）|
| `change_work_frame_cmd` / `get_curr_workFrame_cmd` | Bool / String | 无（软件坐标系注册表）|
| `change_tool_frame_cmd` / `get_current_tool_frame_cmd` | Bool / String | 无 |
| `get_all_work_frame_cmd` / `get_all_tool_frame_cmd` | Getallframe | 无 |
| `set_realtime_push_cmd` / `get_realtime_push_cmd` | Bool / Setrealtimepush | 无（读写内部上报配置）|

---

## 8. 打桩接口（本臂无对应硬件，**不与任何电机交互**）

以下接口话题存在（保证与 rm 接口一致），但硬件不存在：命令类回 `result=false`，查询类回默认（`state=false`）。**均不产生任何 CAN 电机动作**。

- **六维力**：`clear_force_data` / `get_force_data`(含 zero/work/tool 四路) / `start|stop_force_position_move` / `force_position_move[_joint|_pose]` / `set_force_postion` / `stop_force_postion`
- **夹爪**：`set_gripper_pick_on` / `set_gripper_pick` / `set_gripper_position`
- **灵巧手**：`set_hand_posture|seq|angle|speed|force|follow_angle|follow_pos`
- **升降关节**：`set_lift_speed|height` / `get_lift_state`
- **扩展关节**：`set_expand_speed|pos` / `get_expand_state`
- **工具/系统**：`set_tool_voltage` / `clear_system_err`(回 true)
- **Modbus/RS485（四代控制器组）**：`set|get_controller_rs485_mode` / `set|get_tool_rs485_mode` / `add|update|delete|get_modbus_tcp_master` / `get_modbus_tcp_master_list` / `close_controller_rtu|tcp_modbus` / `set_controller_tcp_mode` / `read|write_modbus_rtu|tcp_*`
- **轨迹文件/在线编程**：`get_trajectory_file_list` / `set_run_trajectory` / `delete|save_trajectory_file` / `send_project` / `get_program_run_state` / `get_flowchart_program_run_state`
- **从不发布的 udp_ 状态**：`udp_six_force`/`udp_six_zero_force`/`udp_one_force`/`udp_one_zero_force`/`udp_hand_status`/`udp_arm_current_status`/`udp_arm_coordinate`/`udp_rm_plus_base`/`udp_rm_plus_state`/`udp_lift_state`/`udp_expand_state`/`udp_aloha_state`

---

## 9. 速查：哪些接口动哪些电机

- **动全部 5 个电机**：movej、movel、movec、movej_p、movel_offset、movej_canfd、movep_canfd(+custom)、move_stop、emergency_stop、pause、continue、pos_teach、ort_teach
- **只动 1 个电机**：`set_joint_teach`（`num` 指定）、`set_joint_err_clear`（`joint_num` 指定）
- **只读电机（不驱动）**：get_current_arm_state/original_state、全部 `udp_joint_*` 状态话题
- **完全不碰电机**：版本查询、坐标系、realtime_push、全部第 8 节打桩接口
