// 达妙协议编解码单元测试
#include <gtest/gtest.h>

#include "u1_arm_driver/protocol/damiao_protocol.hpp"

namespace proto = u1_arm::protocol;

using proto::CtrlMode;
using proto::MotorLimits;

// ---- float <-> uint 线性映射 ----

TEST(FloatUint, BoundsAndMidpoint)
{
  // 16 位: min -> 0, max -> 65535, 0 -> 中点
  EXPECT_EQ(proto::float_to_uint(-12.5f, -12.5f, 12.5f, 16), 0u);
  EXPECT_EQ(proto::float_to_uint(12.5f, -12.5f, 12.5f, 16), 65535u);
  const uint32_t mid = proto::float_to_uint(0.0f, -12.5f, 12.5f, 16);
  EXPECT_NEAR(static_cast<double>(mid), 32767.5, 0.5);

  // 12 位
  EXPECT_EQ(proto::float_to_uint(-30.0f, -30.0f, 30.0f, 12), 0u);
  EXPECT_EQ(proto::float_to_uint(30.0f, -30.0f, 30.0f, 12), 4095u);
}

TEST(FloatUint, ClampOutOfRange)
{
  EXPECT_EQ(proto::float_to_uint(-100.0f, -12.5f, 12.5f, 16), 0u);
  EXPECT_EQ(proto::float_to_uint(100.0f, -12.5f, 12.5f, 16), 65535u);
}

TEST(FloatUint, RoundTrip)
{
  // 编码->解码误差应小于 1 LSB 对应的物理量
  const float lsb16 = 25.0f / 65535.0f;
  const float lsb12 = 60.0f / 4095.0f;
  for (float x : {-12.5f, -3.14159f, -0.5f, 0.0f, 0.7f, 3.14159f, 12.5f}) {
    const uint32_t u = proto::float_to_uint(x, -12.5f, 12.5f, 16);
    EXPECT_NEAR(proto::uint_to_float(u, -12.5f, 12.5f, 16), x, lsb16);
  }
  for (float v : {-30.0f, -1.0f, 0.0f, 2.5f, 30.0f}) {
    const uint32_t u = proto::float_to_uint(v, -30.0f, 30.0f, 12);
    EXPECT_NEAR(proto::uint_to_float(u, -30.0f, 30.0f, 12), v, lsb12);
  }
}

// ---- 特殊命令帧 ----

TEST(SpecialFrames, EnableDisablePerMode)
{
  // 教程示例: CAN ID=0x01, 位置速度模式 -> 报文 ID 0x101
  auto en = proto::encode_enable(0x01, CtrlMode::kPosVel);
  EXPECT_EQ(en.id, 0x101u);
  EXPECT_EQ(en.dlc, 8);
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(en.data[i], 0xFF);
  }
  EXPECT_EQ(en.data[7], 0xFC);

  auto dis = proto::encode_disable(0x01, CtrlMode::kMit);
  EXPECT_EQ(dis.id, 0x001u);
  EXPECT_EQ(dis.data[7], 0xFD);

  auto zero = proto::encode_save_zero(0x03, CtrlMode::kVel);
  EXPECT_EQ(zero.id, 0x203u);
  EXPECT_EQ(zero.data[7], 0xFE);

  auto clr = proto::encode_clear_error(0x05, CtrlMode::kForcePos);
  EXPECT_EQ(clr.id, 0x305u);
  EXPECT_EQ(clr.data[7], 0xFB);
}

// ---- 位置速度控制帧 ----

TEST(PosVelFrame, FloatLittleEndianLayout)
{
  auto f = proto::encode_pos_vel(0x02, 1.0f, -2.0f);
  EXPECT_EQ(f.id, 0x102u);
  EXPECT_EQ(f.dlc, 8);
  // float 1.0 = 0x3F800000 小端 -> 00 00 80 3F
  EXPECT_EQ(f.data[0], 0x00);
  EXPECT_EQ(f.data[1], 0x00);
  EXPECT_EQ(f.data[2], 0x80);
  EXPECT_EQ(f.data[3], 0x3F);
  // float -2.0 = 0xC0000000 小端 -> 00 00 00 C0
  EXPECT_EQ(f.data[4], 0x00);
  EXPECT_EQ(f.data[5], 0x00);
  EXPECT_EQ(f.data[6], 0x00);
  EXPECT_EQ(f.data[7], 0xC0);
}

// ---- MIT 控制帧 ----

TEST(MitFrame, BitPackingRoundTrip)
{
  const MotorLimits lim{12.5f, 30.0f, 10.0f};
  const float pos = 1.2f, vel = -3.4f, kp = 10.0f, kd = 0.8f, tau = 2.0f;
  auto f = proto::encode_mit(0x04, lim, pos, vel, kp, kd, tau);
  EXPECT_EQ(f.id, 0x004u);
  EXPECT_EQ(f.dlc, 8);

  // 手工拆包并逆映射
  const uint32_t p = (static_cast<uint32_t>(f.data[0]) << 8) | f.data[1];
  const uint32_t v = (static_cast<uint32_t>(f.data[2]) << 4) | (f.data[3] >> 4);
  const uint32_t kp_u = (static_cast<uint32_t>(f.data[3] & 0x0F) << 8) | f.data[4];
  const uint32_t kd_u = (static_cast<uint32_t>(f.data[5]) << 4) | (f.data[6] >> 4);
  const uint32_t t = (static_cast<uint32_t>(f.data[6] & 0x0F) << 8) | f.data[7];

  EXPECT_NEAR(proto::uint_to_float(p, -12.5f, 12.5f, 16), pos, 25.0f / 65535.0f);
  EXPECT_NEAR(proto::uint_to_float(v, -30.0f, 30.0f, 12), vel, 60.0f / 4095.0f);
  EXPECT_NEAR(proto::uint_to_float(kp_u, 0.0f, proto::kMitKpMax, 12), kp, 500.0f / 4095.0f);
  EXPECT_NEAR(proto::uint_to_float(kd_u, 0.0f, proto::kMitKdMax, 12), kd, 5.0f / 4095.0f);
  EXPECT_NEAR(proto::uint_to_float(t, -10.0f, 10.0f, 12), tau, 20.0f / 4095.0f);
}

// ---- 寄存器操作帧 ----

TEST(RegisterFrames, ReadWriteSaveRefresh)
{
  auto rd = proto::encode_register_read(0x01, proto::kRegCtrlMode);
  EXPECT_EQ(rd.id, 0x7FFu);
  EXPECT_EQ(rd.data[0], 0x01);  // CANID_L
  EXPECT_EQ(rd.data[1], 0x00);  // CANID_H
  EXPECT_EQ(rd.data[2], 0x33);
  EXPECT_EQ(rd.data[3], 0x0A);

  // 写 CTRL_MODE=2(位置速度), data 低位在前
  auto wr = proto::encode_register_write(
    0x01, proto::kRegCtrlMode, static_cast<uint32_t>(proto::CtrlModeReg::kPosVel));
  EXPECT_EQ(wr.data[2], 0x55);
  EXPECT_EQ(wr.data[4], 0x02);
  EXPECT_EQ(wr.data[5], 0x00);
  EXPECT_EQ(wr.data[6], 0x00);
  EXPECT_EQ(wr.data[7], 0x00);

  auto sv = proto::encode_register_save(0x01, proto::kRegCtrlMode);
  EXPECT_EQ(sv.data[2], 0xAA);

  auto rf = proto::encode_refresh(0x0F);
  EXPECT_EQ(rf.id, 0x7FFu);
  EXPECT_EQ(rf.data[0], 0x0F);
  EXPECT_EQ(rf.data[2], 0xCC);
  EXPECT_EQ(rf.data[3], 0x00);
}

// ---- 反馈帧解码 ----

TEST(FeedbackDecode, KnownBytes)
{
  const MotorLimits lim{12.5f, 30.0f, 10.0f};
  // 构造: 电机 ID=3, 使能(1), pos=0(中点 0x7FFF≈32767), vel=最大(0xFFF), tau=最小(0)
  uint8_t data[8];
  data[0] = (0x1 << 4) | 0x3;
  data[1] = 0x7F; data[2] = 0xFF;          // POS = 0x7FFF
  data[3] = 0xFF; data[4] = 0xF0;          // VEL = 0xFFF, T 高 4 位 = 0
  data[5] = 0x00;                          // T = 0x000
  data[6] = 40;                            // MOS 40℃
  data[7] = 55;                            // 线圈 55℃

  auto fb = proto::decode_feedback(data, 8, lim);
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->motor_id, 3);
  EXPECT_EQ(fb->status, proto::MotorStatus::kEnabled);
  EXPECT_NEAR(fb->position, 0.0f, 25.0f / 65535.0f);
  EXPECT_NEAR(fb->velocity, 30.0f, 1e-4f);
  EXPECT_NEAR(fb->torque, -10.0f, 1e-4f);
  EXPECT_FLOAT_EQ(fb->t_mos, 40.0f);
  EXPECT_FLOAT_EQ(fb->t_rotor, 55.0f);
}

TEST(FeedbackDecode, EncodeDecodeConsistency)
{
  // 用 float_to_uint 造帧再解码, 数值闭环
  const MotorLimits lim{3.141593f, 45.0f, 20.0f};
  const float pos = 1.57f, vel = -12.3f, tau = 5.5f;
  const uint32_t p = proto::float_to_uint(pos, -lim.p_max, lim.p_max, 16);
  const uint32_t v = proto::float_to_uint(vel, -lim.v_max, lim.v_max, 12);
  const uint32_t t = proto::float_to_uint(tau, -lim.t_max, lim.t_max, 12);

  uint8_t data[8];
  data[0] = (0x1 << 4) | 0x2;
  data[1] = static_cast<uint8_t>(p >> 8);
  data[2] = static_cast<uint8_t>(p);
  data[3] = static_cast<uint8_t>(v >> 4);
  data[4] = static_cast<uint8_t>(((v & 0xF) << 4) | (t >> 8));
  data[5] = static_cast<uint8_t>(t);
  data[6] = 30;
  data[7] = 35;

  auto fb = proto::decode_feedback(data, 8, lim);
  ASSERT_TRUE(fb.has_value());
  EXPECT_NEAR(fb->position, pos, 2.0f * lim.p_max / 65535.0f);
  EXPECT_NEAR(fb->velocity, vel, 2.0f * lim.v_max / 4095.0f);
  EXPECT_NEAR(fb->torque, tau, 2.0f * lim.t_max / 4095.0f);
}

TEST(FeedbackDecode, FaultStatus)
{
  const MotorLimits lim;
  uint8_t data[8] = {0};
  data[0] = (0xC << 4) | 0x1;  // 线圈过温
  auto fb = proto::decode_feedback(data, 8, lim);
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->status, proto::MotorStatus::kCoilOverTemp);
  EXPECT_TRUE(proto::status_is_fault(fb->status));
  EXPECT_FALSE(proto::status_is_fault(proto::MotorStatus::kEnabled));
  EXPECT_FALSE(proto::status_is_fault(proto::MotorStatus::kDisabled));
}

TEST(FeedbackDecode, ShortFrameRejected)
{
  const MotorLimits lim;
  uint8_t data[8] = {0};
  EXPECT_FALSE(proto::decode_feedback(data, 7, lim).has_value());
}

// ---- 寄存器应答解码 ----

TEST(RegisterReplyDecode, ReadReply)
{
  // 电机应答: CANID=0x01, cmd=0x33, RID=0x0A, value=2 (低位在前)
  uint8_t data[8] = {0x01, 0x00, 0x33, 0x0A, 0x02, 0x00, 0x00, 0x00};
  auto r = proto::decode_register_reply(data, 8);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->can_id, 0x01);
  EXPECT_EQ(r->cmd, 0x33);
  EXPECT_EQ(r->rid, 0x0A);
  EXPECT_EQ(r->value, 2u);
}

TEST(RegisterReplyDecode, NonRegisterCmdRejected)
{
  // D2 不是 0x33/0x55/0xAA -> 不是寄存器应答(可能是反馈帧)
  uint8_t data[8] = {0x13, 0x7F, 0xFF, 0x0F, 0xFF, 0x00, 0x28, 0x37};
  EXPECT_FALSE(proto::decode_register_reply(data, 8).has_value());
}

// ---- 型号量程表 ----

TEST(MotorParams, ModelNameNormalization)
{
  auto a = proto::default_limits("DM-J4310-2EC");
  ASSERT_TRUE(a.has_value());
  EXPECT_FLOAT_EQ(a->v_max, 30.0f);

  auto b = proto::default_limits("dm_3507");
  ASSERT_TRUE(b.has_value());
  EXPECT_FLOAT_EQ(b->v_max, 50.0f);

  auto c = proto::default_limits("DM-J4340-2EC");
  ASSERT_TRUE(c.has_value());
  EXPECT_FLOAT_EQ(c->t_max, 28.0f);

  EXPECT_FALSE(proto::default_limits("UNKNOWN-9999").has_value());
}
