// 命令结构体 -> CAN 帧编码
#ifndef AUTORUNNER_DRIVER__PROTOCOL__ENCODER_HPP_
#define AUTORUNNER_DRIVER__PROTOCOL__ENCODER_HPP_

#include <array>
#include <vector>

#include "autorunner_driver/protocol/frames.hpp"

namespace autorunner::protocol
{

// control_id_offset: 0x00 / 0x10 / 0x20 (0x150~0x15F 整体偏移, 配置帧 0x47x 不偏移)
class Encoder
{
public:
  explicit Encoder(uint32_t control_id_offset = 0)
  : offset_(control_id_offset) {}

  RawFrame encode(const MotionCtrlCmd & cmd) const;
  RawFrame encode(const ModeCtrlCmd & cmd) const;
  // 关节目标 -> 0x155/0x156/0x157
  std::array<RawFrame, 3> encode(const JointTargetCmd & cmd) const;
  // 位姿目标 -> 0x152/0x153/0x154
  std::array<RawFrame, 3> encode(const PoseTargetCmd & cmd) const;
  RawFrame encode(const ArcPointCmd & cmd) const;
  RawFrame encode(const MotorEnableCmd & cmd) const;
  RawFrame encode(const JointLimitQueryCmd & cmd) const;
  RawFrame encode(const JointLimitSetCmd & cmd) const;
  RawFrame encode(const JointConfigCmd & cmd) const;
  RawFrame encode(const ParamQueryCmd & cmd) const;
  RawFrame encode(const EndVelAccSetCmd & cmd) const;
  RawFrame encode(const CollisionLevelSetCmd & cmd) const;

private:
  uint32_t offset_;
};

}  // namespace autorunner::protocol

#endif  // AUTORUNNER_DRIVER__PROTOCOL__ENCODER_HPP_
