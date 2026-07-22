// 达妙(Damiao) 电机型号量程表
// 位置/速度/扭矩在 CAN 帧中按 [-MAX, +MAX] 线性映射为定点数,
// 各型号出厂默认量程不同, 且可被调试助手修改 —— 以 yaml 配置为准, 此表仅提供默认值。
#ifndef U1_ARM_DRIVER__PROTOCOL__MOTOR_PARAMS_HPP_
#define U1_ARM_DRIVER__PROTOCOL__MOTOR_PARAMS_HPP_

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <string_view>

namespace u1_arm::protocol
{

// 量程区间对称: pos∈[-p_max,p_max] rad, vel∈[-v_max,v_max] rad/s, tau∈[-t_max,t_max] N·m
struct MotorLimits
{
  float p_max{12.5f};
  float v_max{30.0f};
  float t_max{10.0f};
};

// MIT 模式增益量程 (达妙全系固件一致): kp∈[0,500], kd∈[0,5]
constexpr float kMitKpMax = 500.0f;
constexpr float kMitKdMax = 5.0f;

// 按型号名查默认量程。匹配对字符做归一化(去掉 -_ 转大写), 因此
// "DM-J4310-2EC" / "dm_3507" / "DM4340" 均可命中。
// 注意: 量程需用达妙调试助手对实际电机核对, 这里只是常见出厂默认。
inline std::optional<MotorLimits> default_limits(std::string_view model)
{
  std::string norm;
  norm.reserve(model.size());
  for (char c : model) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      norm.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
  }
  if (norm.find("4310") != std::string::npos) {return MotorLimits{12.5f, 30.0f, 10.0f};}
  if (norm.find("4340") != std::string::npos) {return MotorLimits{12.5f, 10.0f, 28.0f};}
  if (norm.find("3507") != std::string::npos) {return MotorLimits{12.5f, 50.0f, 5.0f};}
  return std::nullopt;
}

}  // namespace u1_arm::protocol

#endif  // U1_ARM_DRIVER__PROTOCOL__MOTOR_PARAMS_HPP_
