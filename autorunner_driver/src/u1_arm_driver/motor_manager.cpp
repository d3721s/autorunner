#include "u1_arm_driver/motor_manager.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace u1_arm
{

namespace proto = protocol;

namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("u1_arm.motor_manager");
}

rclcpp::Clock & log_clock()
{
  static rclcpp::Clock clock(RCL_SYSTEM_TIME);
  return clock;
}

std::string format_states(const std::vector<MotorState> & states)
{
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < states.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << "{online=" << (states[i].online ? "true" : "false")
        << ", enabled=" << (states[i].enabled ? "true" : "false")
        << ", status=" << static_cast<int>(states[i].status)
        << ", pos=" << std::fixed << std::setprecision(4) << states[i].joint_pos
        << ", vel=" << std::fixed << std::setprecision(4) << states[i].joint_vel
        << ", torque=" << std::fixed << std::setprecision(4) << states[i].torque
        << "}";
  }
  out << "]";
  return out.str();
}

}  // namespace

MotorManager::MotorManager(std::vector<MotorConfig> configs, std::shared_ptr<CanBus> bus)
: configs_(std::move(configs)), bus_(std::move(bus)), states_(configs_.size())
{
  RCLCPP_INFO(logger(), "MotorManager 创建完成: motor_count=%zu", configs_.size());
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
  const auto prev = states_[idx];
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
  RCLCPP_DEBUG_THROTTLE(
    logger(), log_clock(), 1000,
    "反馈入库: index=%zu joint=%s status=%d online %s->true enabled %s->%s pos %.4f->%.4f "
    "vel %.4f->%.4f torque %.4f->%.4f",
    idx, configs_[idx].joint_name.c_str(), static_cast<int>(fb->status),
    prev.online ? "true" : "false", prev.enabled ? "true" : "false",
    s.enabled ? "true" : "false", prev.joint_pos, s.joint_pos, prev.joint_vel, s.joint_vel,
    prev.torque, s.torque);
  return true;
}

protocol::CtrlMode MotorManager::arm_mode() const
{
  std::lock_guard<std::mutex> lock(mode_mutex_);
  return arm_mode_;
}

bool MotorManager::enable(size_t i)
{
  RCLCPP_INFO(
    logger(), "变量下发: enable joint=%s index=%zu mode=%d",
    configs_[i].joint_name.c_str(), i, static_cast<int>(arm_mode()));
  return bus_->send(proto::encode_enable(configs_[i].can_id, arm_mode()));
}

bool MotorManager::disable(size_t i)
{
  RCLCPP_INFO(
    logger(), "变量下发: disable joint=%s index=%zu mode=%d",
    configs_[i].joint_name.c_str(), i, static_cast<int>(arm_mode()));
  return bus_->send(proto::encode_disable(configs_[i].can_id, arm_mode()));
}

bool MotorManager::enable_all()
{
  RCLCPP_INFO(logger(), "变量下发: enable_all count=%zu", configs_.size());
  bool ok = true;
  for (size_t i = 0; i < configs_.size(); ++i) {
    ok = enable(i) && ok;
  }
  return ok;
}

bool MotorManager::disable_all()
{
  RCLCPP_INFO(logger(), "变量下发: disable_all count=%zu", configs_.size());
  bool ok = true;
  for (size_t i = 0; i < configs_.size(); ++i) {
    ok = disable(i) && ok;
  }
  return ok;
}

bool MotorManager::clear_error(size_t i)
{
  RCLCPP_INFO(
    logger(), "变量下发: clear_error joint=%s index=%zu mode=%d",
    configs_[i].joint_name.c_str(), i, static_cast<int>(arm_mode()));
  return bus_->send(proto::encode_clear_error(configs_[i].can_id, arm_mode()));
}

bool MotorManager::save_zero(size_t i)
{
  RCLCPP_INFO(
    logger(), "变量下发: save_zero joint=%s index=%zu mode=%d",
    configs_[i].joint_name.c_str(), i, static_cast<int>(arm_mode()));
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
  RCLCPP_INFO(
    logger(), "控制模式切换: %d -> %d", static_cast<int>(old_mode), static_cast<int>(mode));
  for (size_t i = 0; i < configs_.size(); ++i) {
    // 先在旧偏移失能, 写模式寄存器, 再在新偏移使能
    RCLCPP_INFO(
      logger(),
      "变量下发: set_arm_mode step joint=%s index=%zu disable(old=%d)->write_reg->enable(new=%d)",
      configs_[i].joint_name.c_str(), i, static_cast<int>(old_mode), static_cast<int>(mode));
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
  RCLCPP_INFO(logger(), "控制模式切换完成: arm_mode=%d", static_cast<int>(mode));
}

bool MotorManager::write_ctrl_mode(size_t i)
{
  const uint32_t reg = (arm_mode() == protocol::CtrlMode::kMit) ?
    static_cast<uint32_t>(protocol::CtrlModeReg::kMit) :
    static_cast<uint32_t>(protocol::CtrlModeReg::kPosVel);
  RCLCPP_INFO(
    logger(), "变量下发: write_ctrl_mode joint=%s index=%zu reg=%u mode=%d",
    configs_[i].joint_name.c_str(), i, reg, static_cast<int>(arm_mode()));
  return bus_->send(proto::encode_register_write(configs_[i].can_id, proto::kRegCtrlMode, reg));
}

bool MotorManager::drive(size_t i, double joint_pos, double joint_vel, double joint_torque_ff)
{
  const auto & c = configs_[i];
  const double motor_pos = joint_to_motor(i, joint_pos);

  if (arm_mode() == protocol::CtrlMode::kMit) {
    // MIT: P/V/T_ff 均转换到电机侧; T_ff 由重力/负载补偿给出。
    const double motor_vel = c.direction * joint_vel;
    const double motor_torque_ff = c.direction * joint_torque_ff;
    RCLCPP_DEBUG(
      logger(),
      "变量下发: drive MIT joint=%s index=%zu pos=%.4f vel=%.4f kp=%.4f kd=%.4f tau_ff=%.4f "
      "motor_pos=%.4f motor_vel=%.4f",
      c.joint_name.c_str(), i, joint_pos, joint_vel, c.mit_kp, c.mit_kd, joint_torque_ff,
      motor_pos, motor_vel);
    return bus_->send(
      proto::encode_mit(
        c.can_id, c.limits,
        static_cast<float>(motor_pos), static_cast<float>(motor_vel),
        static_cast<float>(c.mit_kp), static_cast<float>(c.mit_kd),
        static_cast<float>(motor_torque_ff)));
  }
  // 位置速度模式: 位置 + |速度| 作为速度上限
  RCLCPP_DEBUG(
    logger(),
    "变量下发: drive PosVel joint=%s index=%zu pos=%.4f vel=%.4f motor_pos=%.4f",
    c.joint_name.c_str(), i, joint_pos, joint_vel, motor_pos);
  return bus_->send(
    proto::encode_pos_vel(
      c.can_id, static_cast<float>(motor_pos), static_cast<float>(std::abs(joint_vel))));
}

bool MotorManager::refresh(size_t i)
{
  RCLCPP_DEBUG(
    logger(), "变量下发: refresh joint=%s index=%zu mode=%d",
    configs_[i].joint_name.c_str(), i, static_cast<int>(arm_mode()));
  return bus_->send(proto::encode_refresh(configs_[i].can_id));
}

MotorState MotorManager::state(size_t i) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto s = states_[i];
  RCLCPP_DEBUG_THROTTLE(
    logger(), log_clock(), 1000,
    "变量读取: state index=%zu joint=%s online=%s enabled=%s status=%d pos=%.4f vel=%.4f torque=%.4f",
    i, configs_[i].joint_name.c_str(), s.online ? "true" : "false",
    s.enabled ? "true" : "false", static_cast<int>(s.status), s.joint_pos, s.joint_vel,
    s.torque);
  return s;
}

std::vector<MotorState> MotorManager::snapshot() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto states = states_;
  RCLCPP_DEBUG_THROTTLE(
    logger(), log_clock(), 1000, "变量读取: snapshot %s", format_states(states).c_str());
  return states;
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
      RCLCPP_WARN(
        logger(),
        "反馈超时: joint=%s index=%zu timeout_ms=%ld state->online=false enabled=false",
        configs_[i].joint_name.c_str(), i, timeout.count());
    }
  }
  if (!stale.empty()) {
    RCLCPP_WARN(
      logger(), "反馈超时电机数量=%zu indices=[%zu...]", stale.size(), stale.front());
  }
  return stale;
}

}  // namespace u1_arm
