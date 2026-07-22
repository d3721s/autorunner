#include "u1_arm_driver/motor_manager.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

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

  // 注意: 不能用 data[2]∈{0x33,0x55,0xAA} 去区分"寄存器应答", 因为反馈帧的
  // data[2]=POS[7:0](位置低字节)可能恰好等于这些值, 会误丢正常反馈。驱动运行期
  // 只发使能/位置速度/0xCC, 电机回的都是标准反馈帧; 寄存器读写只在 motor_setup
  // 独立进程里做。故这里一律按反馈帧解码。
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

protocol::CtrlMode MotorManager::arm_mode() const
{
  std::lock_guard<std::mutex> lock(mode_mutex_);
  return arm_mode_;
}

bool MotorManager::enable(size_t i)
{
  return bus_->send(proto::encode_enable(configs_[i].can_id, arm_mode()));
}

bool MotorManager::disable(size_t i)
{
  return bus_->send(proto::encode_disable(configs_[i].can_id, arm_mode()));
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
  return bus_->send(proto::encode_clear_error(configs_[i].can_id, arm_mode()));
}

bool MotorManager::save_zero(size_t i)
{
  return bus_->send(proto::encode_save_zero(configs_[i].can_id, arm_mode()));
}

void MotorManager::set_arm_mode(protocol::CtrlMode mode)
{
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (arm_mode_ == mode) {return;}
  }
  // CtrlMode 偏移 -> CTRL_MODE 寄存器编码
  const uint32_t reg = (mode == protocol::CtrlMode::kMit) ?
    static_cast<uint32_t>(protocol::CtrlModeReg::kMit) :
    static_cast<uint32_t>(protocol::CtrlModeReg::kPosVel);

  const auto old_mode = arm_mode();
  for (size_t i = 0; i < configs_.size(); ++i) {
    // 先在旧偏移失能, 写模式寄存器, 再在新偏移使能
    bus_->send(proto::encode_disable(configs_[i].can_id, old_mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    bus_->send(proto::encode_register_write(configs_[i].can_id, proto::kRegCtrlMode, reg));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    bus_->send(proto::encode_enable(configs_[i].can_id, mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    arm_mode_ = mode;
  }
}

bool MotorManager::write_ctrl_mode(size_t i)
{
  const uint32_t reg = (arm_mode() == protocol::CtrlMode::kMit) ?
    static_cast<uint32_t>(protocol::CtrlModeReg::kMit) :
    static_cast<uint32_t>(protocol::CtrlModeReg::kPosVel);
  return bus_->send(proto::encode_register_write(configs_[i].can_id, proto::kRegCtrlMode, reg));
}

bool MotorManager::drive(size_t i, double joint_pos, double joint_vel)
{
  const auto & c = configs_[i];
  const double clamped = std::clamp(joint_pos, c.limit_lower, c.limit_upper);
  const double motor_pos = joint_to_motor(i, clamped);

  if (arm_mode() == protocol::CtrlMode::kMit) {
    // MIT: 位置 P=目标, 速度前馈 V=direction*关节速度, T_ff=0
    const double motor_vel = c.direction * joint_vel;
    return bus_->send(
      proto::encode_mit(
        c.can_id, c.limits,
        static_cast<float>(motor_pos), static_cast<float>(motor_vel),
        static_cast<float>(c.mit_kp), static_cast<float>(c.mit_kd), 0.0f));
  }
  // 位置速度模式: 位置 + |速度| 作为速度上限
  return bus_->send(
    proto::encode_pos_vel(
      c.can_id, static_cast<float>(motor_pos), static_cast<float>(std::abs(joint_vel))));
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
