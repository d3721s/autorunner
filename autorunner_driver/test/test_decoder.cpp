#include <gtest/gtest.h>

#include <cmath>

#include "autorunner_driver/protocol/decoder.hpp"
#include "autorunner_driver/protocol/endian.hpp"

namespace proto = autorunner::protocol;

TEST(DecoderTest, ArmStatus)
{
  proto::Decoder dec;
  // ctrl_mode=1(CAN), arm_status=7(碰撞), move_mode=1(MOVE J),
  // byte6 角度超限位 J3(bit2), byte7 通信异常 J1(bit0)
  const uint8_t data[8] = {0x01, 0x07, 0x01, 0x00, 0x01, 0x05, 0x04, 0x01};
  const auto decoded = dec.decode(0x2A1, data, 8);

  ASSERT_TRUE(std::holds_alternative<proto::ArmStatusFb>(decoded));
  const auto & fb = std::get<proto::ArmStatusFb>(decoded);
  EXPECT_EQ(fb.ctrl_mode, 0x01);
  EXPECT_EQ(fb.arm_status, 0x07);
  EXPECT_EQ(fb.move_mode, 0x01);
  EXPECT_EQ(fb.motion_status, 0x01);
  EXPECT_EQ(fb.trajectory_num, 0x05);
  EXPECT_EQ(fb.joint_angle_limit_err, 0x04);
  EXPECT_EQ(fb.joint_comm_err, 0x01);
}

TEST(DecoderTest, JointAnglePart)
{
  proto::Decoder dec;
  // J1 = 10° = 10000 = 0x00002710, J2 = 0
  uint8_t data[8] = {};
  proto::put_i32_be(data, 10000);
  const auto decoded = dec.decode(0x2A5, data, 8);

  ASSERT_TRUE(std::holds_alternative<proto::JointAnglePart>(decoded));
  const auto & part = std::get<proto::JointAnglePart>(decoded);
  EXPECT_EQ(part.part, 0);
  EXPECT_NEAR(part.a, 10.0 * proto::kPi / 180.0, 1e-9);
  EXPECT_NEAR(part.b, 0.0, 1e-12);
}

TEST(DecoderTest, EndPosePartScales)
{
  proto::Decoder dec;
  uint8_t data[8] = {};
  // 0x2A3: Z(0.001mm) / RX(0.001°) 混合缩放
  proto::put_i32_be(data, 500000);      // Z = 0.5m
  proto::put_i32_be(data + 4, 180000);  // RX = 180°
  const auto decoded = dec.decode(0x2A3, data, 8);

  ASSERT_TRUE(std::holds_alternative<proto::EndPosePart>(decoded));
  const auto & part = std::get<proto::EndPosePart>(decoded);
  EXPECT_EQ(part.part, 1);
  EXPECT_NEAR(part.a, 0.5, 1e-9);
  EXPECT_NEAR(part.b, proto::kPi, 1e-9);
}

TEST(DecoderTest, DriverHighSpeed)
{
  proto::Decoder dec;
  uint8_t data[8] = {};
  proto::put_i16_be(data, -1500);       // -1.5 rad/s
  proto::put_u16_be(data + 2, 2000);    // 2.0 A
  proto::put_i32_be(data + 4, 3141);    // 3.141 rad (0.001rad 假设)
  const auto decoded = dec.decode(0x253, data, 8);

  ASSERT_TRUE(std::holds_alternative<proto::DriverHighSpeedFb>(decoded));
  const auto & fb = std::get<proto::DriverHighSpeedFb>(decoded);
  EXPECT_EQ(fb.joint_index, 2);  // 0x253 -> J3
  EXPECT_NEAR(fb.speed, -1.5, 1e-9);
  EXPECT_NEAR(fb.current, 2.0, 1e-9);
  EXPECT_NEAR(fb.position, 3.141, 1e-9);
}

TEST(DecoderTest, DriverLowSpeedStatusBits)
{
  proto::Decoder dec;
  uint8_t data[8] = {};
  proto::put_u16_be(data, 480);         // 48.0 V
  proto::put_i16_be(data + 2, 45);      // 45 ℃
  data[4] = static_cast<uint8_t>(-10);  // 电机温度 -10℃ (int8)
  data[5] = 0x40;                       // bit6 使能
  proto::put_u16_be(data + 6, 1500);    // 1.5 A
  const auto decoded = dec.decode(0x266, data, 8);

  ASSERT_TRUE(std::holds_alternative<proto::DriverLowSpeedFb>(decoded));
  const auto & fb = std::get<proto::DriverLowSpeedFb>(decoded);
  EXPECT_EQ(fb.joint_index, 5);  // 0x266 -> J6
  EXPECT_NEAR(fb.voltage, 48.0, 1e-9);
  EXPECT_NEAR(fb.driver_temp, 45.0, 1e-9);
  EXPECT_NEAR(fb.motor_temp, -10.0, 1e-9);
  EXPECT_EQ(fb.status, 0x40);
  EXPECT_NEAR(fb.bus_current, 1.5, 1e-9);
}

TEST(DecoderTest, JointLimitFb)
{
  proto::Decoder dec;
  uint8_t data[8] = {};
  data[0] = 4;
  proto::put_i16_be(data + 1, 1800);   // 180.0°
  proto::put_i16_be(data + 3, -1800);  // -180.0°
  proto::put_u16_be(data + 5, 300);    // 3.0 rad/s
  const auto decoded = dec.decode(0x473, data, 8);

  ASSERT_TRUE(std::holds_alternative<proto::JointLimitFb>(decoded));
  const auto & fb = std::get<proto::JointLimitFb>(decoded);
  EXPECT_EQ(fb.joint_num, 4);
  EXPECT_NEAR(fb.max_angle, proto::kPi, 1e-9);
  EXPECT_NEAR(fb.min_angle, -proto::kPi, 1e-9);
  EXPECT_NEAR(fb.max_speed, 3.0, 1e-9);
}

TEST(DecoderTest, SetResponse)
{
  proto::Decoder dec;
  const uint8_t data[8] = {0x75, 0x01, 0, 0, 0, 0, 0, 0};
  const auto decoded = dec.decode(0x476, data, 8);

  ASSERT_TRUE(std::holds_alternative<proto::SetResponseFb>(decoded));
  const auto & fb = std::get<proto::SetResponseFb>(decoded);
  EXPECT_EQ(fb.cmd_index, 0x75);
  EXPECT_TRUE(fb.zero_set_success);
}

TEST(DecoderTest, GripperIgnored)
{
  proto::Decoder dec;
  const uint8_t data[8] = {};
  EXPECT_TRUE(std::holds_alternative<std::monostate>(dec.decode(0x2A8, data, 8)));
}

TEST(DecoderTest, UnknownIdIgnored)
{
  proto::Decoder dec;
  const uint8_t data[8] = {};
  EXPECT_TRUE(std::holds_alternative<std::monostate>(dec.decode(0x123, data, 8)));
}

TEST(DecoderTest, ShortDlcIgnored)
{
  proto::Decoder dec;
  const uint8_t data[8] = {};
  EXPECT_TRUE(std::holds_alternative<std::monostate>(dec.decode(0x2A1, data, 4)));
}

TEST(DecoderTest, FeedbackIdOffset)
{
  // 偏移 0x10: 0x2B1 应解码为机械臂状态
  proto::Decoder dec(0x10);
  const uint8_t data[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_TRUE(std::holds_alternative<proto::ArmStatusFb>(dec.decode(0x2B1, data, 8)));
  // 基址 0x2A1 在偏移模式下不再匹配
  EXPECT_TRUE(std::holds_alternative<std::monostate>(dec.decode(0x2A1, data, 8)));
}
