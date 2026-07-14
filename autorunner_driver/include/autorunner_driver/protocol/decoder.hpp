// CAN 帧 -> 反馈结构体解码
#ifndef AUTORUNNER_DRIVER__PROTOCOL__DECODER_HPP_
#define AUTORUNNER_DRIVER__PROTOCOL__DECODER_HPP_

#include <cstdint>
#include <optional>
#include <variant>

#include "autorunner_driver/protocol/frames.hpp"

namespace autorunner::protocol
{

// 末端位姿/关节角的单帧半包 (组装前的中间产物, 交给 assembler)
struct EndPosePart
{
  uint8_t part{0};   // 0: X/Y  1: Z/RX  2: RY/RZ
  double a{0};       // 第一个 int32 (SI)
  double b{0};       // 第二个 int32 (SI)
};

struct JointAnglePart
{
  uint8_t part{0};   // 0: J1/J2  1: J3/J4  2: J5/J6
  double a{0};       // rad
  double b{0};       // rad
};

using DecodedFrame = std::variant<
  std::monostate,          // 未知/忽略的帧 (含 0x2A8 夹爪)
  ArmStatusFb,             // 0x2A1
  EndPosePart,             // 0x2A2~4
  JointAnglePart,          // 0x2A5~7
  DriverHighSpeedFb,       // 0x251~6
  DriverLowSpeedFb,        // 0x261~6
  JointLimitFb,            // 0x473
  JointMaxAccFb,           // 0x47C
  CollisionLevelFb,        // 0x47B
  EndVelAccFb,             // 0x478
  SetResponseFb>;          // 0x476

// feedback_id_offset: 0x00 / 0x10 / 0x20 (0x2A1~0x2A8 整体偏移)
class Decoder
{
public:
  explicit Decoder(uint32_t feedback_id_offset = 0)
  : offset_(feedback_id_offset) {}

  DecodedFrame decode(uint32_t id, const uint8_t * data, uint8_t dlc) const;

private:
  uint32_t offset_;
};

}  // namespace autorunner::protocol

#endif  // AUTORUNNER_DRIVER__PROTOCOL__DECODER_HPP_
