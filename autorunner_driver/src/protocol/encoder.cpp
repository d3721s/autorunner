#include "autorunner_driver/protocol/encoder.hpp"

#include <cmath>

#include "autorunner_driver/protocol/endian.hpp"
#include "autorunner_driver/protocol/frame_ids.hpp"

namespace autorunner::protocol
{

RawFrame Encoder::encode(const MotionCtrlCmd & cmd) const
{
  RawFrame f;
  f.id = kIdMotionCtrl + offset_;
  f.data[0] = cmd.emergency_stop;
  f.data[1] = cmd.trajectory_ctrl;
  f.data[2] = cmd.drag_teach;
  // Byte3~7 离线轨迹传输字段, 本期填 0
  return f;
}

RawFrame Encoder::encode(const ModeCtrlCmd & cmd) const
{
  RawFrame f;
  f.id = kIdModeCtrl + offset_;
  f.data[0] = cmd.ctrl_mode;
  f.data[1] = cmd.move_mode;
  f.data[2] = cmd.speed;
  f.data[3] = cmd.mit_mode;
  f.data[4] = 0;  // 离线轨迹点停留时间
  f.data[5] = cmd.install_pos;
  return f;
}

std::array<RawFrame, 3> Encoder::encode(const JointTargetCmd & cmd) const
{
  std::array<RawFrame, 3> frames;
  for (int i = 0; i < 3; ++i) {
    frames[i].id = kIdJointCtrl12 + i + offset_;
    put_i32_be(frames[i].data.data(), to_fixed_i32(cmd.joint[i * 2], kMilliDeg));
    put_i32_be(frames[i].data.data() + 4, to_fixed_i32(cmd.joint[i * 2 + 1], kMilliDeg));
  }
  return frames;
}

std::array<RawFrame, 3> Encoder::encode(const PoseTargetCmd & cmd) const
{
  std::array<RawFrame, 3> frames;
  frames[0].id = kIdMoveXy + offset_;
  put_i32_be(frames[0].data.data(), to_fixed_i32(cmd.x, kMicroMeter));
  put_i32_be(frames[0].data.data() + 4, to_fixed_i32(cmd.y, kMicroMeter));
  frames[1].id = kIdMoveZrx + offset_;
  put_i32_be(frames[1].data.data(), to_fixed_i32(cmd.z, kMicroMeter));
  put_i32_be(frames[1].data.data() + 4, to_fixed_i32(cmd.rx, kMilliDeg));
  frames[2].id = kIdMoveRyrz + offset_;
  put_i32_be(frames[2].data.data(), to_fixed_i32(cmd.ry, kMilliDeg));
  put_i32_be(frames[2].data.data() + 4, to_fixed_i32(cmd.rz, kMilliDeg));
  return frames;
}

RawFrame Encoder::encode(const ArcPointCmd & cmd) const
{
  RawFrame f;
  f.id = kIdArcPoint + offset_;
  f.dlc = 1;
  f.data[0] = cmd.point_index;
  return f;
}

RawFrame Encoder::encode(const MotorEnableCmd & cmd) const
{
  RawFrame f;
  f.id = kIdMotorEnable;
  f.data[0] = cmd.joint_num;
  f.data[1] = cmd.enable ? 0x02 : 0x01;
  return f;
}

RawFrame Encoder::encode(const JointLimitQueryCmd & cmd) const
{
  RawFrame f;
  f.id = kIdJointLimitQuery;
  f.data[0] = cmd.joint_num;
  f.data[1] = cmd.query_type;
  return f;
}

RawFrame Encoder::encode(const JointLimitSetCmd & cmd) const
{
  RawFrame f;
  f.id = kIdJointLimitSet;
  f.data[0] = cmd.joint_num;
  put_i16_be(f.data.data() + 1, to_fixed_i16_or_invalid(cmd.max_angle, kDeciDeg));
  put_i16_be(f.data.data() + 3, to_fixed_i16_or_invalid(cmd.min_angle, kDeciDeg));
  put_u16_be(f.data.data() + 5, to_fixed_u16_or_invalid(cmd.max_speed, kCentiRad));
  return f;
}

RawFrame Encoder::encode(const JointConfigCmd & cmd) const
{
  RawFrame f;
  f.id = kIdJointConfig;
  f.data[0] = cmd.joint_num;
  f.data[1] = cmd.set_zero ? 0xAE : 0x00;
  const bool acc_valid = !std::isnan(cmd.max_acc);
  f.data[2] = acc_valid ? 0xAE : 0x00;
  put_u16_be(
    f.data.data() + 3,
    acc_valid ? to_fixed_u16_or_invalid(cmd.max_acc, kCentiRad) :
    static_cast<uint16_t>(kInvalid16));
  f.data[5] = cmd.clear_err ? 0xAE : 0x00;
  return f;
}

RawFrame Encoder::encode(const ParamQueryCmd & cmd) const
{
  RawFrame f;
  f.id = kIdParamQuery;
  f.data[0] = cmd.query_type;
  return f;
}

RawFrame Encoder::encode(const EndVelAccSetCmd & cmd) const
{
  RawFrame f;
  f.id = kIdEndVelAccSet;
  put_u16_be(f.data.data(), to_fixed_u16_or_invalid(cmd.max_linear_vel, kMilli));
  put_u16_be(f.data.data() + 2, to_fixed_u16_or_invalid(cmd.max_angular_vel, kMilli));
  put_u16_be(f.data.data() + 4, to_fixed_u16_or_invalid(cmd.max_linear_acc, kMilli));
  put_u16_be(f.data.data() + 6, to_fixed_u16_or_invalid(cmd.max_angular_acc, kMilli));
  return f;
}

RawFrame Encoder::encode(const CollisionLevelSetCmd & cmd) const
{
  RawFrame f;
  f.id = kIdCollisionSet;
  for (int i = 0; i < 6; ++i) {
    f.data[i] = cmd.level[i];
  }
  return f;
}

}  // namespace autorunner::protocol
