// 达妙(Damiao) 电机 CAN 协议编解码 —— 纯 C++, 零 ROS 依赖
//
// 协议来源: 《达妙电机上手流程(一)-电机的简单调试》(工作区顶层目录)
//   - 控制帧 ID = 电机 CAN ID + 模式偏移 (MIT +0x000 / 位置速度 +0x100 /
//     速度 +0x200 / 力位混控 +0x300)
//   - 使能 FF..FC / 失能 FF..FD / 保存零点 FF..FE / 清除错误 FF..FB
//   - 寄存器操作: 广播 0x7FF, cmd 0x33 读 / 0x55 写 / 0xAA 存 / 0xCC 读电机反馈
//   - 反馈帧(ID=master_id): D0=ID|(ERR<<4), POS16, VEL12, T12 线性映射,
//     D6=MOS 温度℃, D7=线圈温度℃
// 多字节浮点均为小端(与达妙官方例程一致); 定点字段按位拼接见各函数注释。
#ifndef U1_ARM_DRIVER__PROTOCOL__DAMIAO_PROTOCOL_HPP_
#define U1_ARM_DRIVER__PROTOCOL__DAMIAO_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <optional>

#include "u1_arm_driver/protocol/motor_params.hpp"

namespace u1_arm::protocol
{

// ---- 控制模式与其报文 ID 偏移 ----
enum class CtrlMode : uint16_t
{
  kMit = 0x000,
  kPosVel = 0x100,
  kVel = 0x200,
  kForcePos = 0x300,
};

// CTRL_MODE 寄存器(RID 0x0A)的取值编码, 与 CtrlMode 偏移是两套编号
enum class CtrlModeReg : uint32_t
{
  kMit = 1,
  kPosVel = 2,
  kVel = 3,
  kForcePos = 4,
};

// 常用寄存器地址(完整表见官方《调试助手使用说明书(达妙驱动控制协议)》)
constexpr uint8_t kRegCtrlMode = 0x0A;   // 控制模式 [1,4]
constexpr uint8_t kRegCanBaud = 0x23;    // CAN 波特率编码 [0,4](V13)
constexpr uint32_t kBroadcastId = 0x7FF; // 寄存器操作广播 ID

// 电机状态码 (反馈帧 D0 高 4 位)
enum class MotorStatus : uint8_t
{
  kDisabled = 0x0,
  kEnabled = 0x1,
  kOverVoltage = 0x8,
  kUnderVoltage = 0x9,
  kOverCurrent = 0xA,
  kMosOverTemp = 0xB,
  kCoilOverTemp = 0xC,
  kCommLost = 0xD,
  kOverload = 0xE,
};

inline bool status_is_fault(MotorStatus s)
{
  return s != MotorStatus::kDisabled && s != MotorStatus::kEnabled;
}

// 一帧待发送的 CAN 报文 (标准帧)
struct CanFrame
{
  uint32_t id{0};
  uint8_t dlc{8};
  std::array<uint8_t, 8> data{};
};

// 反馈帧解码结果 (物理量)
struct Feedback
{
  uint8_t motor_id{0};       // D0 低 4 位 = CAN ID 低 4 位
  MotorStatus status{MotorStatus::kDisabled};
  float position{0.0f};      // rad
  float velocity{0.0f};      // rad/s
  float torque{0.0f};        // N·m
  float t_mos{0.0f};         // 驱动 MOS 温度 ℃
  float t_rotor{0.0f};       // 电机线圈温度 ℃
};

// 寄存器读/写应答帧解码结果
struct RegisterReply
{
  uint16_t can_id{0};
  uint8_t cmd{0};        // 0x33 / 0x55 / 0xAA
  uint8_t rid{0};
  uint32_t value{0};     // 低位在前
};

// ---- float <-> 定点 线性映射 (MIT 系电机通用套路) ----
// value ∈ [vmin, vmax] 线性映射到 [0, 2^bits - 1]
uint32_t float_to_uint(float value, float vmin, float vmax, unsigned bits);
float uint_to_float(uint32_t raw, float vmin, float vmax, unsigned bits);

// ---- 特殊命令帧 (数据 FF FF FF FF FF FF FF + 末字节) ----
CanFrame encode_enable(uint16_t can_id, CtrlMode mode);        // ..FC
CanFrame encode_disable(uint16_t can_id, CtrlMode mode);       // ..FD
CanFrame encode_save_zero(uint16_t can_id, CtrlMode mode);     // ..FE 保存当前位置为零点
CanFrame encode_clear_error(uint16_t can_id, CtrlMode mode);   // ..FB 清除错误

// ---- 控制帧 ----
// 位置速度模式 (ID+0x100): D0-3=float32 位置 rad, D4-7=float32 速度 rad/s, 小端
CanFrame encode_pos_vel(uint16_t can_id, float pos_rad, float vel_rad_s);

// MIT 模式 (ID+0x000): POS16|VEL12|KP12|KD12|TAU12 大端位拼接:
//   D0=P[15:8] D1=P[7:0] D2=V[11:4] D3=V[3:0]<<4|KP[11:8] D4=KP[7:0]
//   D5=KD[11:4] D6=KD[3:0]<<4|T[11:8] D7=T[7:0]
CanFrame encode_mit(
  uint16_t can_id, const MotorLimits & lim,
  float pos_rad, float vel_rad_s, float kp, float kd, float tau_nm);

// 速度模式 (ID+0x200): D0-3=float32 速度 rad/s 小端
CanFrame encode_vel(uint16_t can_id, float vel_rad_s);

// ---- 寄存器操作帧 (广播 0x7FF) ----
CanFrame encode_register_read(uint16_t can_id, uint8_t rid);
CanFrame encode_register_write(uint16_t can_id, uint8_t rid, uint32_t value);
CanFrame encode_register_save(uint16_t can_id, uint8_t rid);
// 读电机反馈(V13+): 电机以标准反馈帧应答, 用于失能状态下查询位置
CanFrame encode_refresh(uint16_t can_id);

// ---- 解码 ----
// 反馈帧: 任意模式下电机对控制/使能/0xCC 的应答, ID 应等于该电机 master_id。
// 调用方按 frame_id 找到对应电机后传入其量程。dlc<8 返回 nullopt。
std::optional<Feedback> decode_feedback(
  const uint8_t * data, uint8_t dlc, const MotorLimits & lim);

// 寄存器读/写应答帧 (ID=master_id, D2=0x33/0x55/0xAA)。
// 与反馈帧共用 master_id, 靠 D2 命令字节区分 —— 先试此函数, 失败再按反馈帧解。
std::optional<RegisterReply> decode_register_reply(const uint8_t * data, uint8_t dlc);

}  // namespace u1_arm::protocol

#endif  // U1_ARM_DRIVER__PROTOCOL__DAMIAO_PROTOCOL_HPP_
