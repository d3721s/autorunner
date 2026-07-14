// 与 CAN 帧一一对应的 POD 结构体, SI 单位, 零 ROS 依赖
#ifndef AUTORUNNER_DRIVER__PROTOCOL__FRAMES_HPP_
#define AUTORUNNER_DRIVER__PROTOCOL__FRAMES_HPP_

#include <array>
#include <cstdint>

namespace autorunner::protocol
{

// 编码输出: 一条待发送的 CAN 帧
struct RawFrame
{
  uint32_t id{0};
  uint8_t dlc{8};
  std::array<uint8_t, 8> data{};
};

// ---------------- 反馈方向 ----------------

// 0x2A1 机械臂状态反馈
struct ArmStatusFb
{
  uint8_t ctrl_mode{0};
  uint8_t arm_status{0};
  uint8_t move_mode{0};
  uint8_t teach_status{0};
  uint8_t motion_status{0};
  uint8_t trajectory_num{0};
  uint8_t joint_comm_err{0};        // byte7 bit0~5
  uint8_t joint_angle_limit_err{0}; // byte6 bit0~5
};

// 0x2A2~4 组装后的末端位姿 (m / rad)
struct EndPoseFb
{
  double x{0}, y{0}, z{0};
  double rx{0}, ry{0}, rz{0};
};

// 0x2A5~7 组装后的关节角 (rad)
struct JointAngleFb
{
  std::array<double, 6> joint{};
};

// 0x251~6 单关节高速反馈
struct DriverHighSpeedFb
{
  uint8_t joint_index{0};  // 0~5
  double speed{0};         // rad/s
  double current{0};       // A
  double position{0};      // rad (缩放待实机确认)
};

// 0x261~6 单关节低速反馈
struct DriverLowSpeedFb
{
  uint8_t joint_index{0};  // 0~5
  double voltage{0};       // V
  double driver_temp{0};   // ℃
  double motor_temp{0};    // ℃
  uint8_t status{0};       // 状态位字节
  double bus_current{0};   // A
};

// 0x473 电机限制反馈
struct JointLimitFb
{
  uint8_t joint_num{0};    // 1~6
  double max_angle{0};     // rad
  double min_angle{0};     // rad
  double max_speed{0};     // rad/s
};

// 0x47C 电机最大加速度反馈
struct JointMaxAccFb
{
  uint8_t joint_num{0};
  double max_acc{0};       // rad/s^2
};

// 0x47B 碰撞防护等级反馈
struct CollisionLevelFb
{
  std::array<uint8_t, 6> level{};
};

// 0x478 末端速度/加速度反馈
struct EndVelAccFb
{
  double max_linear_vel{0};   // m/s
  double max_angular_vel{0};  // rad/s
  double max_linear_acc{0};   // m/s^2
  double max_angular_acc{0};  // rad/s^2
};

// 0x476 设置指令应答
struct SetResponseFb
{
  uint8_t cmd_index{0};
  bool zero_set_success{false};
};

// ---------------- 命令方向 ----------------

// 0x150
struct MotionCtrlCmd
{
  uint8_t emergency_stop{0};   // 0无效 1急停 2恢复
  uint8_t trajectory_ctrl{0};
  uint8_t drag_teach{0};
};

// 0x151
struct ModeCtrlCmd
{
  uint8_t ctrl_mode{0};
  uint8_t move_mode{0};
  uint8_t speed{0};        // 0~100
  uint8_t mit_mode{0};     // 0x00 / 0xAD
  uint8_t install_pos{0};
};

// 目标关节角 (rad) -> 0x155/156/157 三帧
struct JointTargetCmd
{
  std::array<double, 6> joint{};
};

// 目标末端位姿 (m/rad) -> 0x152/153/154 三帧
struct PoseTargetCmd
{
  double x{0}, y{0}, z{0};
  double rx{0}, ry{0}, rz{0};
};

// 0x158
struct ArcPointCmd
{
  uint8_t point_index{0};  // 1起点 2中点 3终点
};

// 0x471
struct MotorEnableCmd
{
  uint8_t joint_num{0};    // 1~6, 7全部
  bool enable{false};
};

// 0x472
struct JointLimitQueryCmd
{
  uint8_t joint_num{0};
  uint8_t query_type{0};   // 1角度/速度 2最大加速度
};

// 0x474 (NaN = 不修改 -> 0x7FFF)
struct JointLimitSetCmd
{
  uint8_t joint_num{0};
  double max_angle{0};     // rad
  double min_angle{0};     // rad
  double max_speed{0};     // rad/s
};

// 0x475 (max_acc NaN = 不设置)
struct JointConfigCmd
{
  uint8_t joint_num{0};
  bool set_zero{false};
  bool clear_err{false};
  double max_acc{0};       // rad/s^2
};

// 0x477 参数查询 (仅开放 Byte0)
struct ParamQueryCmd
{
  uint8_t query_type{0};   // 1末端V/acc 2碰撞防护等级
};

// 0x479 (NaN = 不修改)
struct EndVelAccSetCmd
{
  double max_linear_vel{0};
  double max_angular_vel{0};
  double max_linear_acc{0};
  double max_angular_acc{0};
};

// 0x47A
struct CollisionLevelSetCmd
{
  std::array<uint8_t, 6> level{};
};

}  // namespace autorunner::protocol

#endif  // AUTORUNNER_DRIVER__PROTOCOL__FRAMES_HPP_
