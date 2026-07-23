#include "u1_arm_driver/trajectory_executor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace u1_arm
{

namespace
{

constexpr double kArrivalToleranceRad = 0.03;
constexpr auto kArrivalPollPeriod = std::chrono::milliseconds(10);

rclcpp::Logger logger()
{
  return rclcpp::get_logger("u1_arm.trajectory_executor");
}

rclcpp::Clock & log_clock()
{
  static rclcpp::Clock clock(RCL_SYSTEM_TIME);
  return clock;
}

std::string format_vector(const std::vector<double> & values, int precision = 4)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << values[i];
  }
  out << "]";
  return out.str();
}

}  // namespace

TrajectoryExecutor::TrajectoryExecutor(std::shared_ptr<MotorManager> mgr, double tick_period_s)
: mgr_(std::move(mgr)), dt_(tick_period_s)
{
  const size_t n = mgr_->size();
  hold_.assign(n, 0.0);
  prev_hold_.assign(n, 0.0);
  vel_limit_.assign(n, 1.0);
  RCLCPP_INFO(logger(), "TrajectoryExecutor 创建完成: joints=%zu dt=%.6f", n, dt_);
}

void TrajectoryExecutor::set_torque_feedforward(TorqueFeedforwardFn fn)
{
  std::lock_guard<std::mutex> lock(mutex_);
  torque_ff_fn_ = std::move(fn);
  RCLCPP_INFO(logger(), "变量更新: torque_feedforward_fn 已设置");
}

void TrajectoryExecutor::hold_from_feedback_locked()
{
  const auto states = mgr_->snapshot();
  for (size_t i = 0; i < states.size(); ++i) {
    hold_[i] = states[i].joint_pos;
  }
  prev_hold_ = hold_;   // 保持位不产生前馈速度
  hold_valid_ = true;
  RCLCPP_INFO(logger(), "变量更新: hold_from_feedback hold=%s", format_vector(hold_).c_str());
}

void TrajectoryExecutor::finish_motion_locked(bool completed)
{
  const auto old_mode = mode_;
  mode_ = Mode::kIdle;
  paused_ = false;
  ++motion_seq_;
  last_motion_completed_ = completed;
  RCLCPP_INFO(
    logger(),
    "运动结束: mode %d->%d completed=%s motion_seq=%lu paused=false",
    static_cast<int>(old_mode), static_cast<int>(mode_), completed ? "true" : "false",
    motion_seq_);
  done_cv_.notify_all();
}

bool TrajectoryExecutor::feedback_matches_hold_locked() const
{
  const auto states = mgr_->snapshot();
  if (states.size() != hold_.size()) {return false;}
  for (size_t i = 0; i < states.size(); ++i) {
    const auto & s = states[i];
    if (!s.online || !s.enabled || !std::isfinite(s.joint_pos)) {return false;}
    if (std::abs(s.joint_pos - hold_[i]) > kArrivalToleranceRad) {return false;}
  }
  return true;
}

bool TrajectoryExecutor::start_movej(const std::vector<double> & goal, uint8_t speed_percent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(
    logger(), "start_movej 请求: goal=%s speed=%u estopped=%s hold_size=%zu",
    format_vector(goal).c_str(), speed_percent, estopped_ ? "true" : "false", hold_.size());
  if (estopped_ || goal.size() != hold_.size()) {
    RCLCPP_WARN(
      logger(), "start_movej 拒绝: estopped=%s goal_size=%zu hold_size=%zu",
      estopped_ ? "true" : "false", goal.size(), hold_.size());
    return false;
  }

  // 目标限位校验
  for (size_t i = 0; i < goal.size(); ++i) {
    const auto & c = mgr_->config(i);
    if (!std::isfinite(goal[i]) ||
      goal[i] < c.limit_lower - 1e-6 || goal[i] > c.limit_upper + 1e-6)
    {
      RCLCPP_WARN(
        logger(), "start_movej 拒绝: index=%zu goal=%.4f limit=[%.4f, %.4f]",
        i, goal[i], c.limit_lower, c.limit_upper);
      return false;
    }
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {
    if (!mgr_->all_seen()) {
      RCLCPP_WARN(logger(), "start_movej 拒绝: 尚未收到全部电机反馈");
      return false;
    }
    hold_from_feedback_locked();
  }

  const double ratio = std::clamp(static_cast<double>(speed_percent), 1.0, 100.0) / 100.0;

  // 全关节同步: 取各关节按自身 vmax/amax 所需时间的最大值
  // 五次多项式峰值速度 = 1.875*|Δ|/T, 峰值加速度 = 5.774*|Δ|/T²
  double duration = 0.1;
  for (size_t i = 0; i < goal.size(); ++i) {
    const auto & c = mgr_->config(i);
    const double d = std::abs(goal[i] - hold_[i]);
    if (d < 1e-9) {continue;}
    const double vmax = std::max(1e-3, c.vmax * ratio);
    const double amax = std::max(1e-3, c.amax * ratio);
    duration = std::max(duration, 1.875 * d / vmax);
    duration = std::max(duration, std::sqrt(5.774 * d / amax));
  }

  quintic_.start = hold_;
  quintic_.delta.resize(goal.size());
  for (size_t i = 0; i < goal.size(); ++i) {
    quintic_.delta[i] = goal[i] - hold_[i];
  }
  quintic_.duration = duration;
  quintic_.t = 0.0;

  // 下发速度限幅取峰值速度的 1.2 倍(电机内环余量)
  for (size_t i = 0; i < goal.size(); ++i) {
    vel_limit_[i] = std::max(0.1, 1.875 * std::abs(quintic_.delta[i]) / duration * 1.2);
  }
  mode_ = Mode::kTrajectory;
  paused_ = false;
  RCLCPP_INFO(
    logger(),
    "变量更新: mode=kTrajectory paused=false quintic_duration=%.4f start=%s delta=%s "
    "vel_limit=%s",
    quintic_.duration, format_vector(quintic_.start).c_str(),
    format_vector(quintic_.delta).c_str(), format_vector(vel_limit_).c_str());
  return true;
}

bool TrajectoryExecutor::start_sampled(std::vector<std::vector<double>> points)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(
    logger(), "start_sampled 请求: points=%zu estopped=%s",
    points.size(), estopped_ ? "true" : "false");
  if (estopped_ || points.empty()) {
    RCLCPP_WARN(
      logger(), "start_sampled 拒绝: estopped=%s points=%zu", estopped_ ? "true" : "false",
      points.size());
    return false;
  }
  for (const auto & p : points) {
    if (p.size() != hold_.size()) {
      RCLCPP_WARN(
        logger(), "start_sampled 拒绝: point_size=%zu hold_size=%zu",
        p.size(), hold_.size());
      return false;
    }
    for (size_t i = 0; i < p.size(); ++i) {
      const auto & c = mgr_->config(i);
      if (!std::isfinite(p[i]) || p[i] < c.limit_lower - 1e-6 || p[i] > c.limit_upper + 1e-6) {
        RCLCPP_WARN(
          logger(), "start_sampled 拒绝: index=%zu value=%.4f limit=[%.4f, %.4f]",
          i, p[i], c.limit_lower, c.limit_upper);
        return false;
      }
    }
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {
    if (!mgr_->all_seen()) {
      RCLCPP_WARN(logger(), "start_sampled 拒绝: 尚未收到全部电机反馈");
      return false;
    }
    hold_from_feedback_locked();
  }
  prev_hold_ = hold_;

  sampled_.points = std::move(points);
  sampled_.cursor = 0;
  // 采样轨迹相邻点间隔 dt, 速度限幅按相邻步长动态估计, 初值给宽
  for (size_t i = 0; i < vel_limit_.size(); ++i) {
    vel_limit_[i] = mgr_->config(i).vmax * 1.2;
  }
  mode_ = Mode::kSampled;
  paused_ = false;
  RCLCPP_INFO(
    logger(), "变量更新: mode=kSampled paused=false sampled_points=%zu vel_limit=%s",
    sampled_.points.size(), format_vector(vel_limit_).c_str());
  return true;
}

bool TrajectoryExecutor::start_jog(size_t joint_index, int dir, uint8_t speed_percent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(
    logger(), "start_jog 请求: joint_index=%zu dir=%d speed=%u estopped=%s",
    joint_index, dir, speed_percent, estopped_ ? "true" : "false");
  if (estopped_ || joint_index >= hold_.size() || dir == 0) {
    RCLCPP_WARN(
      logger(), "start_jog 拒绝: estopped=%s joint_index=%zu hold_size=%zu dir=%d",
      estopped_ ? "true" : "false", joint_index, hold_.size(), dir);
    return false;
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {
    if (!mgr_->all_seen()) {
      RCLCPP_WARN(logger(), "start_jog 拒绝: 尚未收到全部电机反馈");
      return false;
    }
    hold_from_feedback_locked();
  }

  const double ratio = std::clamp(static_cast<double>(speed_percent), 1.0, 100.0) / 100.0;
  jog_.joint = joint_index;
  jog_.dir = dir > 0 ? 1 : -1;
  jog_.vel = mgr_->config(joint_index).vmax * ratio;
  prev_hold_ = hold_;
  vel_limit_[joint_index] = jog_.vel * 1.2;
  mode_ = Mode::kJog;
  paused_ = false;
  RCLCPP_INFO(
    logger(),
    "变量更新: mode=kJog paused=false jog_joint=%zu jog_dir=%d jog_vel=%.4f vel_limit[%zu]=%.4f",
    jog_.joint, jog_.dir, jog_.vel, joint_index, vel_limit_[joint_index]);
  return true;
}

void TrajectoryExecutor::stop_jog()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == Mode::kJog) {
    prev_hold_ = hold_;
    finish_motion_locked(true);
    RCLCPP_INFO(logger(), "stop_jog: 已停止 jog hold=%s", format_vector(hold_).c_str());
  } else {
    RCLCPP_INFO(logger(), "stop_jog: 当前 mode=%d, 无 jog 可停止", static_cast<int>(mode_));
  }
}

bool TrajectoryExecutor::passthrough(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(
    logger(), "passthrough 请求: joints=%s estopped=%s",
    format_vector(joints).c_str(), estopped_ ? "true" : "false");
  if (estopped_ || joints.size() != hold_.size()) {
    RCLCPP_WARN(
      logger(), "passthrough 拒绝: estopped=%s joints_size=%zu hold_size=%zu",
      estopped_ ? "true" : "false", joints.size(), hold_.size());
    return false;
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto & c = mgr_->config(i);
    if (!std::isfinite(joints[i])) {return false;}
    hold_[i] = std::clamp(joints[i], c.limit_lower, c.limit_upper);
    vel_limit_[i] = c.vmax * 1.2;   // 透传由上游保证平滑, 限幅给宽
  }
  prev_hold_ = hold_;
  hold_valid_ = true;
  RCLCPP_INFO(
    logger(), "变量更新: passthrough hold=%s vel_limit=%s hold_valid=true",
    format_vector(hold_).c_str(), format_vector(vel_limit_).c_str());
  return true;
}

void TrajectoryExecutor::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  prev_hold_ = hold_;
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  // hold_ 停在最后 setpoint, tick 继续下发 -> 电机原地保持
  RCLCPP_INFO(logger(), "stop: prev_hold=hold=%s", format_vector(hold_).c_str());
}

void TrajectoryExecutor::pause()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == Mode::kTrajectory || mode_ == Mode::kSampled) {
    paused_ = true;
    RCLCPP_INFO(logger(), "变量更新: paused_=true mode=%d", static_cast<int>(mode_));
  } else {
    RCLCPP_INFO(logger(), "pause 忽略: mode=%d", static_cast<int>(mode_));
  }
}

void TrajectoryExecutor::resume()
{
  std::lock_guard<std::mutex> lock(mutex_);
  paused_ = false;
  RCLCPP_INFO(logger(), "变量更新: paused_=false mode=%d", static_cast<int>(mode_));
}

void TrajectoryExecutor::estop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  estopped_ = true;
  prev_hold_ = hold_;
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  RCLCPP_WARN(
    logger(), "变量更新: estopped_=true prev_hold=%s mode=%d",
    format_vector(prev_hold_).c_str(), static_cast<int>(mode_));
}

void TrajectoryExecutor::release_estop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  estopped_ = false;
  // 解除后以当前反馈为保持位, 避免急停期间被推动导致跳变
  if (mgr_->all_seen()) {
    hold_from_feedback_locked();
  } else {
    hold_valid_ = false;
  }
  RCLCPP_INFO(
    logger(), "变量更新: estopped_=false hold_valid=%s",
    hold_valid_ ? "true" : "false");
}

bool TrajectoryExecutor::estopped() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return estopped_;
}

bool TrajectoryExecutor::busy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_ != Mode::kIdle;
}

bool TrajectoryExecutor::wait_motion_done(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  RCLCPP_INFO(
    logger(), "wait_motion_done 开始: timeout_ms=%ld mode=%d motion_seq=%lu",
    timeout.count(), static_cast<int>(mode_), motion_seq_);
  if (mode_ != Mode::kIdle) {
    const uint64_t seq = motion_seq_;
    if (!done_cv_.wait_until(lock, deadline, [&] {return motion_seq_ != seq;})) {
      RCLCPP_WARN(logger(), "wait_motion_done 超时: 等待 motion_seq 变化失败");
      return false;
    }
  }
  if (!last_motion_completed_) {
    RCLCPP_WARN(logger(), "wait_motion_done 失败: last_motion_completed_=false");
    return false;
  }

  while (!feedback_matches_hold_locked()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      RCLCPP_WARN(
        logger(), "wait_motion_done 超时: feedback 未匹配 hold=%s",
        format_vector(hold_).c_str());
      return false;
    }
    auto wake = now + kArrivalPollPeriod;
    if (wake > deadline) {wake = deadline;}
    if (done_cv_.wait_until(lock, wake, [&] {return mode_ != Mode::kIdle || estopped_;})) {
      RCLCPP_WARN(
        logger(), "wait_motion_done 中断: mode=%d estopped=%s",
        static_cast<int>(mode_), estopped_ ? "true" : "false");
      return false;
    }
  }
  RCLCPP_INFO(logger(), "wait_motion_done 成功: hold=%s", format_vector(hold_).c_str());
  return true;
}

std::vector<double> TrajectoryExecutor::commanded() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_DEBUG_THROTTLE(
    logger(), log_clock(), 1000, "变量读取: commanded hold=%s", format_vector(hold_).c_str());
  return hold_;
}

void TrajectoryExecutor::tick()
{
  std::unique_lock<std::mutex> lock(mutex_);

  // 首次: 等到全部电机有反馈后锁存保持位
  if (!hold_valid_) {
    if (!mgr_->all_seen()) {
      lock.unlock();
      for (size_t i = 0; i < mgr_->size(); ++i) {
        mgr_->refresh(i);
      }
      RCLCPP_DEBUG_THROTTLE(logger(), log_clock(), 1000, "tick: 等待首帧反馈, 已下发 refresh");
      return;
    }
    hold_from_feedback_locked();
  }

  std::vector<double> planned_vel(hold_.size(), 0.0);
  bool has_planned_vel = false;

  if (!paused_) {
    switch (mode_) {
      case Mode::kIdle:
        break;
      case Mode::kTrajectory: {
          quintic_.t = std::min(quintic_.t + dt_, quintic_.duration);
          const double tau = quintic_.t / quintic_.duration;
          const double s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
          const double ds_dt =
            (30.0 * tau * tau - 60.0 * tau * tau * tau + 30.0 * tau * tau * tau * tau) /
            quintic_.duration;
          for (size_t i = 0; i < hold_.size(); ++i) {
            hold_[i] = quintic_.start[i] + quintic_.delta[i] * s;
            planned_vel[i] = quintic_.delta[i] * ds_dt;
          }
          has_planned_vel = true;
          if (quintic_.t >= quintic_.duration) {finish_motion_locked(true);}
          RCLCPP_DEBUG_THROTTLE(
            logger(), log_clock(), 500,
            "变量更新: trajectory t=%.4f/%.4f hold=%s planned_vel=%s",
            quintic_.t, quintic_.duration, format_vector(hold_).c_str(),
            format_vector(planned_vel).c_str());
          break;
        }
      case Mode::kSampled: {
          hold_ = sampled_.points[sampled_.cursor];
          if (++sampled_.cursor >= sampled_.points.size()) {finish_motion_locked(true);}
          RCLCPP_DEBUG_THROTTLE(
            logger(), log_clock(), 500,
            "变量更新: sampled cursor=%zu/%zu hold=%s",
            sampled_.cursor, sampled_.points.size(), format_vector(hold_).c_str());
          break;
        }
      case Mode::kJog: {
          const auto & c = mgr_->config(jog_.joint);
          double p = hold_[jog_.joint] + jog_.dir * jog_.vel * dt_;
          p = std::clamp(p, c.limit_lower, c.limit_upper);
          hold_[jog_.joint] = p;
          // 到限位自动停
          if (p <= c.limit_lower + 1e-9 || p >= c.limit_upper - 1e-9) {
            finish_motion_locked(true);
          }
          RCLCPP_DEBUG_THROTTLE(
            logger(), log_clock(), 500,
            "变量更新: jog joint=%zu hold=%.4f vel=%.4f",
            jog_.joint, hold_[jog_.joint], jog_.vel);
          break;
        }
    }
  }

  // 计算下发速度: MoveJ 使用五次多项式解析速度作 MIT 前馈;
  // 采样轨迹/jog 使用位置差分; 位置速度模式(canfd)用 vel_limit_ 作速度上限。
  const bool mit = (mgr_->arm_mode() == protocol::CtrlMode::kMit);
  const auto targets = hold_;
  const auto torque_ff_fn = torque_ff_fn_;
  std::vector<double> vels(hold_.size());
  for (size_t i = 0; i < hold_.size(); ++i) {
    vels[i] = mit ? (has_planned_vel ? planned_vel[i] : (hold_[i] - prev_hold_[i]) / dt_) :
      vel_limit_[i];
  }
  prev_hold_ = hold_;
  const auto tick_mode = mode_;
  const bool tick_paused = paused_;
  const bool tick_estopped = estopped_;
  lock.unlock();

  std::vector<double> torques(targets.size(), 0.0);
  if (mit && torque_ff_fn) {
    try {
      auto computed = torque_ff_fn(targets);
      if (computed.size() == targets.size()) {
        torques = std::move(computed);
      }
    } catch (const std::exception &) {
      // 前馈失败时退化为无力矩前馈, 保持位置/速度闭环继续工作。
    }
  }

  // 下发: 使能电机按当前模式发控制帧(一发一收, 附带引出反馈), 未使能电机发 0xCC 查询
  const auto states = mgr_->snapshot();
  for (size_t i = 0; i < targets.size(); ++i) {
    if (states[i].enabled) {
      mgr_->drive(i, targets[i], vels[i], torques[i]);
    } else {
      mgr_->refresh(i);
    }
  }
  RCLCPP_DEBUG_THROTTLE(
    logger(), log_clock(), 500,
    "tick 下发完成: mode=%d paused=%s estopped=%s targets=%s vels=%s torques=%s",
    static_cast<int>(tick_mode), tick_paused ? "true" : "false", tick_estopped ? "true" : "false",
    format_vector(targets).c_str(), format_vector(vels).c_str(), format_vector(torques).c_str());
}

}  // namespace u1_arm
