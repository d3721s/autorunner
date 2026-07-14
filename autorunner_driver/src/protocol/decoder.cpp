#include "autorunner_driver/protocol/decoder.hpp"

#include "autorunner_driver/protocol/endian.hpp"
#include "autorunner_driver/protocol/frame_ids.hpp"

namespace autorunner::protocol
{

DecodedFrame Decoder::decode(uint32_t id, const uint8_t * data, uint8_t dlc) const
{
  if (dlc < 8) {
    // 本驱动关心的反馈帧均为 8 字节
    return std::monostate{};
  }

  // 反馈帧按偏移还原基址
  const uint32_t fb_id = id - offset_;

  switch (fb_id) {
    case kIdArmStatus: {
        ArmStatusFb fb;
        fb.ctrl_mode = data[0];
        fb.arm_status = data[1];
        fb.move_mode = data[2];
        fb.teach_status = data[3];
        fb.motion_status = data[4];
        fb.trajectory_num = data[5];
        fb.joint_angle_limit_err = data[6];
        fb.joint_comm_err = data[7];
        return fb;
      }
    case kIdEndPose1:
    case kIdEndPose2:
    case kIdEndPose3: {
        EndPosePart part;
        part.part = static_cast<uint8_t>(fb_id - kIdEndPose1);
        // 0x2A2: X/Y (0.001mm)  0x2A3: Z(0.001mm)/RX(0.001°)  0x2A4: RY/RZ (0.001°)
        const double scale_a = (part.part <= 1) ? kMicroMeter : kMilliDeg;
        const double scale_b = (part.part == 0) ? kMicroMeter : kMilliDeg;
        part.a = get_i32_be(data) * scale_a;
        part.b = get_i32_be(data + 4) * scale_b;
        return part;
      }
    case kIdJointFb12:
    case kIdJointFb34:
    case kIdJointFb56: {
        JointAnglePart part;
        part.part = static_cast<uint8_t>(fb_id - kIdJointFb12);
        part.a = get_i32_be(data) * kMilliDeg;
        part.b = get_i32_be(data + 4) * kMilliDeg;
        return part;
      }
    case kIdGripperFb:
      // 无夹爪, 忽略
      return std::monostate{};
    default:
      break;
  }

  // 无偏移的帧
  if (id >= kIdDriverHighBase && id < kIdDriverHighBase + 6) {
    DriverHighSpeedFb fb;
    fb.joint_index = static_cast<uint8_t>(id - kIdDriverHighBase);
    fb.speed = get_i16_be(data) * kMilli;
    fb.current = get_u16_be(data + 2) * kMilli;
    fb.position = get_i32_be(data + 4) * kMotorPosScale;
    return fb;
  }
  if (id >= kIdDriverLowBase && id < kIdDriverLowBase + 6) {
    DriverLowSpeedFb fb;
    fb.joint_index = static_cast<uint8_t>(id - kIdDriverLowBase);
    fb.voltage = get_u16_be(data) * kDeciVolt;
    fb.driver_temp = static_cast<double>(get_i16_be(data + 2));
    fb.motor_temp = static_cast<double>(static_cast<int8_t>(data[4]));
    fb.status = data[5];
    fb.bus_current = get_u16_be(data + 6) * kMilli;
    return fb;
  }

  switch (id) {
    case kIdJointLimitFb: {
        JointLimitFb fb;
        fb.joint_num = data[0];
        fb.max_angle = get_i16_be(data + 1) * kDeciDeg;
        fb.min_angle = get_i16_be(data + 3) * kDeciDeg;
        fb.max_speed = get_u16_be(data + 5) * kCentiRad;
        return fb;
      }
    case kIdJointMaxAccFb: {
        JointMaxAccFb fb;
        fb.joint_num = data[0];
        // 协议 0x47C 标注单位 0.001rad/s^2
        fb.max_acc = get_u16_be(data + 1) * kMilli;
        return fb;
      }
    case kIdCollisionFb: {
        CollisionLevelFb fb;
        for (int i = 0; i < 6; ++i) {
          fb.level[i] = data[i];
        }
        return fb;
      }
    case kIdEndVelAccFb: {
        EndVelAccFb fb;
        fb.max_linear_vel = get_u16_be(data) * kMilli;
        fb.max_angular_vel = get_u16_be(data + 2) * kMilli;
        fb.max_linear_acc = get_u16_be(data + 4) * kMilli;
        fb.max_angular_acc = get_u16_be(data + 6) * kMilli;
        return fb;
      }
    case kIdSetResponse: {
        SetResponseFb fb;
        fb.cmd_index = data[0];
        fb.zero_set_success = data[1] == 0x01;
        return fb;
      }
    default:
      return std::monostate{};
  }
}

}  // namespace autorunner::protocol
