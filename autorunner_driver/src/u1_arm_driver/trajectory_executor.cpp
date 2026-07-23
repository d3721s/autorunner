#include "u1_arm_driver/trajectory_executor.hpp"

#include <algorithm>
#include <cmath>

namespace u1_arm
{

namespace
{

constexpr double kArrivalToleranceRad = 0.03;
constexpr auto kArrivalPollPeriod = std::chrono::milliseconds(10);

}  // namespace

TrajectoryExecutor::TrajectoryExecutor(std::shared_ptr<MotorManager> mgr, double tick_period_s)
: mgr_(std::move(mgr)), dt_(tick_period_s)
{
  const size_t n = mgr_->size();
  hold_.assign(n, 0.0);
  prev_hold_.assign(n, 0.0);
  vel_limit_.assign(n, 1.0);
}

void TrajectoryExecutor::set_torque_feedforward(TorqueFeedforwardFn fn)
{
  std::lock_guard<std::mutex> lock(mutex_);
  torque_ff_fn_ = std::move(fn);
}

void TrajectoryExecutor::hold_from_feedback_locked()
{
  const auto states = mgr_->snapshot();
  for (size_t i = 0; i < states.size(); ++i) {
    hold_[i] = states[i].joint_pos;
  }
  prev_hold_ = hold_;   // 保持位不产生前馈速度
  hold_valid_ = true;
}

void TrajectoryExecutor::finish_motion_locked(bool completed)
{
  mode_ = Mode::kIdle;
  paused_ = false;
  ++motion_seq_;
  last_motion_completed_ = completed;
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
  if (estopped_ || goal.size() != hold_.size()) {return false;}

  // 目标限位校验
  for (size_t i = 0; i < goal.size(); ++i) {
    const auto & c = mgr_->config(i);
    if (!std::isfinite(goal[i]) ||
      goal[i] < c.limit_lower - 1e-6 || goal[i] > c.limit_upper + 1e-6)
    {
      return false;
    }
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {
    if (!mgr_->all_seen()) {return false;}
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
  return true;
}

bool TrajectoryExecutor::start_sampled(std::vector<std::vector<double>> points)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (estopped_ || points.empty()) {return false;}
  for (const auto & p : points) {
    if (p.size() != hold_.size()) {return false;}
    for (size_t i = 0; i < p.size(); ++i) {
      const auto & c = mgr_->config(i);
      if (!std::isfinite(p[i]) || p[i] < c.limit_lower - 1e-6 || p[i] > c.limit_upper + 1e-6) {
        return false;
      }
    }
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {
    if (!mgr_->all_seen()) {return false;}
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
  return true;
}

bool TrajectoryExecutor::start_jog(size_t joint_index, int dir, uint8_t speed_percent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (estopped_ || joint_index >= hold_.size() || dir == 0) {return false;}
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {
    if (!mgr_->all_seen()) {return false;}
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
  return true;
}

void TrajectoryExecutor::stop_jog()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == Mode::kJog) {
    prev_hold_ = hold_;
    finish_motion_locked(true);
  }
}

bool TrajectoryExecutor::passthrough(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (estopped_ || joints.size() != hold_.size()) {return false;}
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto & c = mgr_->config(i);
    if (!std::isfinite(joints[i])) {return false;}
    hold_[i] = std::clamp(joints[i], c.limit_lower, c.limit_upper);
    vel_limit_[i] = c.vmax * 1.2;   // 透传由上游保证平滑, 限幅给宽
  }
  prev_hold_ = hold_;
  hold_valid_ = true;
  return true;
}

void TrajectoryExecutor::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  prev_hold_ = hold_;
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  // hold_ 停在最后 setpoint, tick 继续下发 -> 电机原地保持
}

void TrajectoryExecutor::pause()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == Mode::kTrajectory || mode_ == Mode::kSampled) {paused_ = true;}
}

void TrajectoryExecutor::resume()
{
  std::lock_guard<std::mutex> lock(mutex_);
  paused_ = false;
}

void TrajectoryExecutor::estop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  estopped_ = true;
  prev_hold_ = hold_;
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
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
  if (mode_ != Mode::kIdle) {
    const uint64_t seq = motion_seq_;
    if (!done_cv_.wait_until(lock, deadline, [&] {return motion_seq_ != seq;})) {
      return false;
    }
  }
  if (!last_motion_completed_) {return false;}

  while (!feedback_matches_hold_locked()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {return false;}
    auto wake = now + kArrivalPollPeriod;
    if (wake > deadline) {wake = deadline;}
    if (done_cv_.wait_until(lock, wake, [&] {return mode_ != Mode::kIdle || estopped_; })) {
      return false;
    }
  }
  return true;
}

std::vector<double> TrajectoryExecutor::commanded() const
{
  std::lock_guard<std::mutex> lock(mutex_);
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
          break;
        }
      case Mode::kSampled: {
          hold_ = sampled_.points[sampled_.cursor];
          if (++sampled_.cursor >= sampled_.points.size()) {finish_motion_locked(true);}
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
}

}  // namespace u1_arm
