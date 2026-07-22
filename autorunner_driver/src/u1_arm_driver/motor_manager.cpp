#include "u1_arm_driver/motor_manager.hpp"

#include <algorithm>

namespace u1_arm
{

namespace proto = protocol;

MotorManager::MotorManager(std::vector<MotorConfig> configs, std::shared_ptr<CanBus> bus)
: configs_(std::move(configs)), bus_(std::move(bus)), states_(configs_.size())
{
}

bool MotorManager::handle_frame(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  // 按 master_id 找电机
  size_t idx = configs_.size();
  for (size_t i = 0; i < configs_.size(); ++i) {
    if (configs_[i].master_id == id) {
      idx = i;
      break;
    }
  }
  if (idx == configs_.size()) {return false;}

  // 寄存器应答(0x33/0x55/0xAA)与反馈帧共用 master_id, 先排除寄存器应答
  if (proto::decode_register_reply(data, dlc).has_value()) {
    return true;  // 寄存器应答由 motor_setup 工具处理, 驱动侧忽略
  }

  const auto fb = proto::decode_feedback(data, dlc, configs_[idx].limits);
  if (!fb.has_value()) {return false;}

  std::lock_guard<std::mutex> lock(state_mutex_);
  auto & s = states_[idx];
  s.online = true;
  s.status = fb->status;
  s.enabled = (fb->status == proto::MotorStatus::kEnabled);
  s.joint_pos = motor_to_joint(idx, fb->position);
  s.joint_vel = configs_[idx].direction * fb->velocity;
  s.torque = fb->torque;
  s.t_mos = fb->t_mos;
  s.t_rotor = fb->t_rotor;
  s.last_feedback = std::chrono::steady_clock::now();
  return true;
}

bool MotorManager::enable(size_t i)
{
  return bus_->send(proto::encode_enable(configs_[i].can_id, proto::CtrlMode::kPosVel));
}

bool MotorManager::disable(size_t i)
{
  return bus_->send(proto::encode_disable(configs_[i].can_id, proto::CtrlMode::kPosVel));
}

bool MotorManager::enable_all()
{
  bool ok = true;
  for (size_t i = 0; i < configs_.size(); ++i) {
    ok = enable(i) && ok;
  }
  return ok;
}

bool MotorManager::disable_all()
{
  bool ok = true;
  for (size_t i = 0; i < configs_.size(); ++i) {
    ok = disable(i) && ok;
  }
  return ok;
}

bool MotorManager::clear_error(size_t i)
{
  return bus_->send(proto::encode_clear_error(configs_[i].can_id, proto::CtrlMode::kPosVel));
}

bool MotorManager::save_zero(size_t i)
{
  return bus_->send(proto::encode_save_zero(configs_[i].can_id, proto::CtrlMode::kPosVel));
}

bool MotorManager::send_pos_vel(size_t i, double joint_pos, double joint_vel_limit)
{
  const auto & c = configs_[i];
  const double clamped = std::clamp(joint_pos, c.limit_lower, c.limit_upper);
  const double motor_pos = joint_to_motor(i, clamped);
  const double vel = std::abs(joint_vel_limit);
  return bus_->send(
    proto::encode_pos_vel(
      c.can_id, static_cast<float>(motor_pos), static_cast<float>(vel)));
}

bool MotorManager::refresh(size_t i)
{
  return bus_->send(proto::encode_refresh(configs_[i].can_id));
}

MotorState MotorManager::state(size_t i) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return states_[i];
}

std::vector<MotorState> MotorManager::snapshot() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return states_;
}

bool MotorManager::all_seen() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::all_of(states_.begin(), states_.end(), [](const auto & s) {return s.online;});
}

std::vector<size_t> MotorManager::check_timeout(std::chrono::milliseconds timeout)
{
  const auto now = std::chrono::steady_clock::now();
  std::vector<size_t> stale;
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < states_.size(); ++i) {
    auto & s = states_[i];
    if (s.online && now - s.last_feedback > timeout) {
      s.online = false;
      s.enabled = false;
      stale.push_back(i);
    }
  }
  return stale;
}

}  // namespace u1_arm
