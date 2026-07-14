// 大端(Motorola MSB)字节序读写 helper 与协议缩放常量
// 协议: CAN2.0B 标准帧, 1M 波特率, 数据格式 Motorola(MSB)
#ifndef AUTORUNNER_DRIVER__PROTOCOL__ENDIAN_HPP_
#define AUTORUNNER_DRIVER__PROTOCOL__ENDIAN_HPP_

#include <cmath>
#include <cstdint>

namespace autorunner::protocol
{

inline int32_t get_i32_be(const uint8_t * p)
{
  return static_cast<int32_t>(
    (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
    (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]));
}

inline int16_t get_i16_be(const uint8_t * p)
{
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

inline uint16_t get_u16_be(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

inline void put_i32_be(uint8_t * p, int32_t v)
{
  const auto u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u >> 24);
  p[1] = static_cast<uint8_t>(u >> 16);
  p[2] = static_cast<uint8_t>(u >> 8);
  p[3] = static_cast<uint8_t>(u);
}

inline void put_i16_be(uint8_t * p, int16_t v)
{
  const auto u = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(u >> 8);
  p[1] = static_cast<uint8_t>(u);
}

inline void put_u16_be(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v);
}

// ---- 协议缩放常量（SI 单位 <-> 协议定点） ----
constexpr double kPi = 3.14159265358979323846;
constexpr double kMilliDeg = 1e-3 * kPi / 180.0;  // 0.001° -> rad
constexpr double kMicroMeter = 1e-6;              // 0.001mm -> m
constexpr double kMilli = 1e-3;                   // 0.001 -> 1 (rad/s, A, N·m, m/s ...)
constexpr double kCentiRad = 1e-2;                // 0.01rad/s(0x473/474) 0.01rad/s^2(0x475)
constexpr double kDeciDeg = 0.1 * kPi / 180.0;    // 0.1° -> rad (0x473/474 角度限制)
constexpr double kDeciVolt = 0.1;                 // 0.1V -> V
// 0x251~6 电机位置: 文档标注 int32 单位 rad, 按同族字段惯例推断为 0.001rad —— 待实机确认
constexpr double kMotorPosScale = 1e-3;
constexpr int16_t kInvalid16 = 0x7FFF;            // "不修改"哨兵

// SI -> 定点 (四舍五入)
inline int32_t to_fixed_i32(double si, double scale)
{
  return static_cast<int32_t>(std::lround(si / scale));
}

// NaN -> 0x7FFF 哨兵, 其余按缩放转 int16/uint16
inline int16_t to_fixed_i16_or_invalid(double si, double scale)
{
  if (std::isnan(si)) {return kInvalid16;}
  return static_cast<int16_t>(std::lround(si / scale));
}

inline uint16_t to_fixed_u16_or_invalid(double si, double scale)
{
  if (std::isnan(si)) {return static_cast<uint16_t>(kInvalid16);}
  return static_cast<uint16_t>(std::lround(si / scale));
}

}  // namespace autorunner::protocol

#endif  // AUTORUNNER_DRIVER__PROTOCOL__ENDIAN_HPP_
