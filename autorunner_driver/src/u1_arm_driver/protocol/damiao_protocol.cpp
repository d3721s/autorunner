// 达妙电机 CAN 协议编解码实现 —— 帧格式见头文件与教程文档
#include "u1_arm_driver/protocol/damiao_protocol.hpp"

#include <cmath>
#include <cstring>

namespace u1_arm::protocol
{

namespace
{

// 特殊命令帧公共部分: FF FF FF FF FF FF FF + tail
CanFrame make_special(uint16_t can_id, CtrlMode mode, uint8_t tail)
{
  CanFrame f;
  f.id = static_cast<uint32_t>(mode) + can_id;
  f.dlc = 8;
  f.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, tail};
  return f;
}

void put_f32_le(uint8_t * p, float v)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  std::memcpy(p, &v, 4);  // 目标平台(x86/arm linux)为小端, 与达妙例程一致
}

float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

uint32_t float_to_uint(float value, float vmin, float vmax, unsigned bits)
{
  const float span = vmax - vmin;
  const float clamped = clampf(value, vmin, vmax);
  const auto max_raw = static_cast<uint32_t>((1u << bits) - 1u);
  const float raw = (clamped - vmin) / span * static_cast<float>(max_raw);
  return static_cast<uint32_t>(std::lround(raw));
}

float uint_to_float(uint32_t raw, float vmin, float vmax, unsigned bits)
{
  const float span = vmax - vmin;
  const auto max_raw = static_cast<uint32_t>((1u << bits) - 1u);
  return static_cast<float>(raw) / static_cast<float>(max_raw) * span + vmin;
}

CanFrame encode_enable(uint16_t can_id, CtrlMode mode)
{
  return make_special(can_id, mode, 0xFC);
}

CanFrame encode_disable(uint16_t can_id, CtrlMode mode)
{
  return make_special(can_id, mode, 0xFD);
}

CanFrame encode_save_zero(uint16_t can_id, CtrlMode mode)
{
  return make_special(can_id, mode, 0xFE);
}

CanFrame encode_clear_error(uint16_t can_id, CtrlMode mode)
{
  return make_special(can_id, mode, 0xFB);
}

CanFrame encode_pos_vel(uint16_t can_id, float pos_rad, float vel_rad_s)
{
  CanFrame f;
  f.id = static_cast<uint32_t>(CtrlMode::kPosVel) + can_id;
  f.dlc = 8;
  put_f32_le(f.data.data(), pos_rad);
  put_f32_le(f.data.data() + 4, vel_rad_s);
  return f;
}

CanFrame encode_mit(
  uint16_t can_id, const MotorLimits & lim,
  float pos_rad, float vel_rad_s, float kp, float kd, float tau_nm)
{
  const uint32_t p = float_to_uint(pos_rad, -lim.p_max, lim.p_max, 16);
  const uint32_t v = float_to_uint(vel_rad_s, -lim.v_max, lim.v_max, 12);
  const uint32_t kp_u = float_to_uint(kp, 0.0f, kMitKpMax, 12);
  const uint32_t kd_u = float_to_uint(kd, 0.0f, kMitKdMax, 12);
  const uint32_t t = float_to_uint(tau_nm, -lim.t_max, lim.t_max, 12);

  CanFrame f;
  f.id = static_cast<uint32_t>(CtrlMode::kMit) + can_id;
  f.dlc = 8;
  f.data[0] = static_cast<uint8_t>(p >> 8);
  f.data[1] = static_cast<uint8_t>(p);
  f.data[2] = static_cast<uint8_t>(v >> 4);
  f.data[3] = static_cast<uint8_t>(((v & 0x0F) << 4) | (kp_u >> 8));
  f.data[4] = static_cast<uint8_t>(kp_u);
  f.data[5] = static_cast<uint8_t>(kd_u >> 4);
  f.data[6] = static_cast<uint8_t>(((kd_u & 0x0F) << 4) | (t >> 8));
  f.data[7] = static_cast<uint8_t>(t);
  return f;
}

CanFrame encode_vel(uint16_t can_id, float vel_rad_s)
{
  CanFrame f;
  f.id = static_cast<uint32_t>(CtrlMode::kVel) + can_id;
  f.dlc = 4;
  put_f32_le(f.data.data(), vel_rad_s);
  return f;
}

CanFrame encode_register_read(uint16_t can_id, uint8_t rid)
{
  CanFrame f;
  f.id = kBroadcastId;
  f.dlc = 8;
  f.data = {static_cast<uint8_t>(can_id & 0xFF), static_cast<uint8_t>(can_id >> 8),
    0x33, rid, 0x00, 0x00, 0x00, 0x00};
  return f;
}

CanFrame encode_register_write(uint16_t can_id, uint8_t rid, uint32_t value)
{
  CanFrame f;
  f.id = kBroadcastId;
  f.dlc = 8;
  f.data[0] = static_cast<uint8_t>(can_id & 0xFF);
  f.data[1] = static_cast<uint8_t>(can_id >> 8);
  f.data[2] = 0x55;
  f.data[3] = rid;
  // data 低位在前
  f.data[4] = static_cast<uint8_t>(value);
  f.data[5] = static_cast<uint8_t>(value >> 8);
  f.data[6] = static_cast<uint8_t>(value >> 16);
  f.data[7] = static_cast<uint8_t>(value >> 24);
  return f;
}

CanFrame encode_register_save(uint16_t can_id, uint8_t rid)
{
  CanFrame f;
  f.id = kBroadcastId;
  f.dlc = 8;
  f.data = {static_cast<uint8_t>(can_id & 0xFF), static_cast<uint8_t>(can_id >> 8),
    0xAA, rid, 0x00, 0x00, 0x00, 0x00};
  return f;
}

CanFrame encode_refresh(uint16_t can_id)
{
  CanFrame f;
  f.id = kBroadcastId;
  f.dlc = 8;
  f.data = {static_cast<uint8_t>(can_id & 0xFF), static_cast<uint8_t>(can_id >> 8),
    0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
  return f;
}

std::optional<Feedback> decode_feedback(
  const uint8_t * data, uint8_t dlc, const MotorLimits & lim)
{
  if (dlc < 8) {return std::nullopt;}

  Feedback fb;
  fb.motor_id = static_cast<uint8_t>(data[0] & 0x0F);
  fb.status = static_cast<MotorStatus>(data[0] >> 4);

  const uint32_t p = (static_cast<uint32_t>(data[1]) << 8) | data[2];
  const uint32_t v = (static_cast<uint32_t>(data[3]) << 4) | (data[4] >> 4);
  const uint32_t t = (static_cast<uint32_t>(data[4] & 0x0F) << 8) | data[5];

  fb.position = uint_to_float(p, -lim.p_max, lim.p_max, 16);
  fb.velocity = uint_to_float(v, -lim.v_max, lim.v_max, 12);
  fb.torque = uint_to_float(t, -lim.t_max, lim.t_max, 12);
  fb.t_mos = static_cast<float>(data[6]);
  fb.t_rotor = static_cast<float>(data[7]);
  return fb;
}

std::optional<RegisterReply> decode_register_reply(const uint8_t * data, uint8_t dlc)
{
  if (dlc < 4) {return std::nullopt;}
  const uint8_t cmd = data[2];
  if (cmd != 0x33 && cmd != 0x55 && cmd != 0xAA) {return std::nullopt;}

  RegisterReply r;
  r.can_id = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
  r.cmd = cmd;
  r.rid = data[3];
  r.value = 0;
  // data 低位在前, dlc 可能为 4~8
  for (int i = 0; i < 4 && 4 + i < dlc; ++i) {
    r.value |= static_cast<uint32_t>(data[4 + i]) << (8 * i);
  }
  return r;
}

}  // namespace u1_arm::protocol
