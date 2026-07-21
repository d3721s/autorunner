#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "autorunner_driver/protocol/encoder.hpp"
#include "autorunner_driver/protocol/endian.hpp"
#include "autorunner_driver/protocol/frame_ids.hpp"

namespace proto = autorunner::protocol;

TEST(EncoderTest, JointTargetGoldenVector)
{
  // J1 = 90° = 1.5708rad -> 90000 = 0x00015F90
  proto::Encoder enc;
  proto::JointTargetCmd cmd;
  cmd.joint[0] = proto::kPi / 2.0;

  const auto frames = enc.encode(cmd);
  ASSERT_EQ(frames[0].id, 0x155u);
  ASSERT_EQ(frames[1].id, 0x156u);
  ASSERT_EQ(frames[2].id, 0x157u);
  EXPECT_EQ(frames[0].dlc, 8);
  EXPECT_EQ(frames[0].data[0], 0x00);
  EXPECT_EQ(frames[0].data[1], 0x01);
  EXPECT_EQ(frames[0].data[2], 0x5F);
  EXPECT_EQ(frames[0].data[3], 0x90);
  // J2~J6 = 0
  for (int i = 4; i < 8; ++i) {
    EXPECT_EQ(frames[0].data[i], 0x00);
  }
}

TEST(EncoderTest, PoseTargetGoldenVector)
{
  // X = -0.1m = -100mm -> -100000 = 0xFFFE7960
  proto::Encoder enc;
  proto::PoseTargetCmd cmd;
  cmd.x = -0.1;

  const auto frames = enc.encode(cmd);
  ASSERT_EQ(frames[0].id, 0x152u);
  EXPECT_EQ(frames[0].data[0], 0xFF);
  EXPECT_EQ(frames[0].data[1], 0xFE);
  EXPECT_EQ(frames[0].data[2], 0x79);
  EXPECT_EQ(frames[0].data[3], 0x60);
}

TEST(EncoderTest, ModeCtrl)
{
  proto::Encoder enc;
  proto::ModeCtrlCmd cmd;
  cmd.ctrl_mode = 0x01;
  cmd.move_mode = 0x01;
  cmd.speed = 20;
  cmd.install_pos = 1;

  const auto f = enc.encode(cmd);
  EXPECT_EQ(f.id, 0x151u);
  EXPECT_EQ(f.data[0], 0x01);
  EXPECT_EQ(f.data[1], 0x01);
  EXPECT_EQ(f.data[2], 20);
  EXPECT_EQ(f.data[3], 0x00);
  EXPECT_EQ(f.data[5], 0x01);
}

TEST(EncoderTest, MotorEnable)
{
  proto::Encoder enc;
  proto::MotorEnableCmd cmd;
  cmd.joint_num = 7;
  cmd.enable = true;

  const auto f = enc.encode(cmd);
  EXPECT_EQ(f.id, 0x471u);
  EXPECT_EQ(f.data[0], 7);
  EXPECT_EQ(f.data[1], 0x02);

  cmd.enable = false;
  EXPECT_EQ(enc.encode(cmd).data[1], 0x01);
}

TEST(EncoderTest, EmergencyStop)
{
  proto::Encoder enc;
  proto::MotionCtrlCmd cmd;
  cmd.emergency_stop = 0x01;

  const auto f = enc.encode(cmd);
  EXPECT_EQ(f.id, 0x150u);
  EXPECT_EQ(f.data[0], 0x01);
  for (int i = 1; i < 8; ++i) {
    EXPECT_EQ(f.data[i], 0x00);
  }
}

TEST(EncoderTest, JointLimitSetNanSentinel)
{
  proto::Encoder enc;
  proto::JointLimitSetCmd cmd;
  cmd.joint_num = 3;
  cmd.max_angle = std::numeric_limits<double>::quiet_NaN();
  cmd.min_angle = std::numeric_limits<double>::quiet_NaN();
  cmd.max_speed = 1.0;  // rad/s -> 0.01rad/s 定点 = 100

  const auto f = enc.encode(cmd);
  EXPECT_EQ(f.id, 0x474u);
  EXPECT_EQ(f.data[0], 3);
  // NaN -> 0x7FFF
  EXPECT_EQ(proto::get_i16_be(f.data.data() + 1), 0x7FFF);
  EXPECT_EQ(proto::get_i16_be(f.data.data() + 3), 0x7FFF);
  EXPECT_EQ(proto::get_u16_be(f.data.data() + 5), 100);
}

TEST(EncoderTest, JointConfigZeroAndClear)
{
  proto::Encoder enc;
  proto::JointConfigCmd cmd;
  cmd.joint_num = 2;
  cmd.set_zero = true;
  cmd.clear_err = true;
  cmd.max_acc = std::numeric_limits<double>::quiet_NaN();

  const auto f = enc.encode(cmd);
  EXPECT_EQ(f.id, 0x475u);
  EXPECT_EQ(f.data[0], 2);
  EXPECT_EQ(f.data[1], 0xAE);       // 设零点
  EXPECT_EQ(f.data[2], 0x00);       // 加速度不生效
  EXPECT_EQ(proto::get_u16_be(f.data.data() + 3), 0x7FFF);
  EXPECT_EQ(f.data[5], 0xAE);       // 清错
}

TEST(EncoderTest, ArcPointDlc1)
{
  proto::Encoder enc;
  proto::ArcPointCmd cmd;
  cmd.point_index = 0x02;

  const auto f = enc.encode(cmd);
  EXPECT_EQ(f.id, 0x158u);
  EXPECT_EQ(f.dlc, 1);
  EXPECT_EQ(f.data[0], 0x02);
}

TEST(EncoderTest, ControlIdOffset)
{
  // 控制指令偏移 0x10: 0x155 -> 0x165; 配置帧 0x471 不偏移
  proto::Encoder enc(0x10);
  proto::JointTargetCmd cmd;
  EXPECT_EQ(enc.encode(cmd)[0].id, 0x165u);

  proto::MotorEnableCmd en;
  en.joint_num = 7;
  en.enable = true;
  EXPECT_EQ(enc.encode(en).id, 0x471u);
}
