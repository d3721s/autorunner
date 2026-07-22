// 电机管理器: 5 个达妙电机的配置、状态缓存与关节侧<->电机侧换算
// - CanBus rx 线程调 handle_frame() 写入反馈缓存
// - 控制循环调 send_pos_vel()/refresh() 下发命令
// - 关节角与电机角换算: motor = direction * joint + zero_offset
#ifndef U1_ARM_DRIVER__MOTOR_MANAGER_HPP_
#define U1_ARM_DRIVER__MOTOR_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "u1_arm_driver/can_bus.hpp"
#include "u1_arm_driver/protocol/damiao_protocol.hpp"

namespace u1_arm
{

struct MotorConfig
{
  std::string joint_name;
  std::string model{"DM-J4310"};
  uint16_t can_id{1};
  uint16_t master_id{0x11};
  protocol::MotorLimits limits;   // 电机定点映射量程 (与电机内部设置一致!)
  int direction{1};               // 关节正向与电机正向关系 +1/-1
  double zero_offset{0.0};        // 电机侧零位偏移 rad
  double limit_lower{-3.14};      // 关节侧软限位 rad
  double limit_upper{3.14};
  double vmax{2.0};               // 关节侧规划限速 rad/s
  double amax{5.0};               // 关节侧规划限加速度 rad/s^2
};

struct MotorState
{
  bool online{false};            // 收到过反馈且未超时
  bool enabled{false};
  protocol::MotorStatus status{protocol::MotorStatus::kDisabled};
  double joint_pos{0.0};         // 关节侧 rad
  double joint_vel{0.0};         // 关节侧 rad/s
  double torque{0.0};            // 电机侧 N·m
  float t_mos{0.0f};
  float t_rotor{0.0f};
  std::chrono::steady_clock::time_point last_feedback{};
};

class MotorManager
{
public:
  MotorManager(std::vector<MotorConfig> configs, std::shared_ptr<CanBus> bus);

  size_t size() const {return configs_.size();}
  const MotorConfig & config(size_t i) const {return configs_[i];}

  // CAN 收帧入口 (rx 线程): 按 master_id 匹配电机并解码缓存。
  // 返回 true 表示该帧已被识别消费。
  bool handle_frame(uint32_t id, const uint8_t * data, uint8_t dlc);

  // ---- 命令 (线程安全, 直接走 CanBus) ----
  bool enable(size_t i);
  bool disable(size_t i);
  bool enable_all();
  bool disable_all();
  bool clear_error(size_t i);      // FB 清错帧
  bool save_zero(size_t i);        // FE 保存当前位置为零点
  // 关节侧目标 -> 位置速度模式帧; 位置裁剪到软限位, vel_limit 取绝对值
  bool send_pos_vel(size_t i, double joint_pos, double joint_vel_limit);
  bool refresh(size_t i);          // 0x7FF/0xCC 查询反馈(失能状态下用)

  // ---- 状态 ----
  MotorState state(size_t i) const;
  std::vector<MotorState> snapshot() const;
  // 全部电机是否已收到过反馈(用于启动时等待首帧)
  bool all_seen() const;
  // 反馈超时判定, 由 watchdog 周期调用; 返回超时电机下标列表
  std::vector<size_t> check_timeout(std::chrono::milliseconds timeout);

  // 关节侧 -> 电机侧
  double joint_to_motor(size_t i, double joint_pos) const
  {
    return configs_[i].direction * joint_pos + configs_[i].zero_offset;
  }
  // 电机侧 -> 关节侧
  double motor_to_joint(size_t i, double motor_pos) const
  {
    return configs_[i].direction * (motor_pos - configs_[i].zero_offset);
  }

private:
  std::vector<MotorConfig> configs_;
  std::shared_ptr<CanBus> bus_;
  mutable std::mutex state_mutex_;
  std::vector<MotorState> states_;
};

}  // namespace u1_arm

#endif  // U1_ARM_DRIVER__MOTOR_MANAGER_HPP_
