#include "u1_arm_driver/trajectory_executor.hpp"

#include <algorithm>
#include <cmath>

namespace u1_arm
{

TrajectoryExecutor::TrajectoryExecutor(std::shared_ptr<MotorManager> mgr, double tick_period_s)
: mgr_(std::move(mgr)), dt_(tick_period_s)
{
  const size_t n = mgr_->size();
  hold_.assign(n, 0.0);
  vel_limit_.assign(n, 1.0);
}

void TrajectoryExecutor::hold_from_feedback_locked()
{
  const auto states = mgr_->snapshot();
  for (size_t i = 0; i < states.size(); ++i) {
    hold_[i] = states[i].joint_pos;
  }
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

bool TrajectoryExecutor::start_movej(const std::vector<double> & goal, uint8_t speed_percent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (estopped_ || goal.size() != hold_.size()) {return false;}

  // 目标限位校验
  for (size_t i = 0; i < goal.size(); ++i) {
    const auto & c = mgr_->config(i);
    if (goal[i] < c.limit_lower - 1e-6 || goal[i] > c.limit_upper + 1e-6) {
      return false;
    }
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {hold_from_feedback_locked();}

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
  }
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  if (!hold_valid_) {hold_from_feedback_locked();}

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
  if (!hold_valid_) {hold_from_feedback_locked();}

  const double ratio = std::clamp(static_cast<double>(speed_percent), 1.0, 100.0) / 100.0;
  jog_.joint = joint_index;
  jog_.dir = dir > 0 ? 1 : -1;
  jog_.vel = mgr_->config(joint_index).vmax * ratio;
  vel_limit_[joint_index] = jog_.vel * 1.2;
  mode_ = Mode::kJog;
  paused_ = false;
  return true;
}

void TrajectoryExecutor::stop_jog()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == Mode::kJog) {finish_motion_locked(true);}
}

bool TrajectoryExecutor::passthrough(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (estopped_ || joints.size() != hold_.size()) {return false;}
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto & c = mgr_->config(i);
    hold_[i] = std::clamp(joints[i], c.limit_lower, c.limit_upper);
    vel_limit_[i] = c.vmax * 1.2;   // 透传由上游保证平滑, 限幅给宽
  }
  hold_valid_ = true;
  return true;
}

void TrajectoryExecutor::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
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
  if (mode_ != Mode::kIdle) {finish_motion_locked(false);}
}

void TrajectoryExecutor::release_estop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  estopped_ = false;
  // 解除后以当前反馈为保持位, 避免急停期间被推动导致跳变
  hold_from_feedback_locked();
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
  std::unique_lock<std::mutex> lock(mutex_);
  if (mode_ == Mode::kIdle) {return last_motion_completed_;}
  const uint64_t seq = motion_seq_;
  if (!done_cv_.wait_for(lock, timeout, [&] {return motion_seq_ != seq;})) {
    return false;
  }
  return last_motion_completed_;
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

  if (!paused_) {
    switch (mode_) {
      case Mode::kIdle:
        break;
      case Mode::kTrajectory: {
          quintic_.t = std::min(quintic_.t + dt_, quintic_.duration);
          const double tau = quintic_.t / quintic_.duration;
          const double s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
          for (size_t i = 0; i < hold_.size(); ++i) {
            hold_[i] = quintic_.start[i] + quintic_.delta[i] * s;
          }
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

  // 下发: 使能电机发位置速度帧(附带引出反馈), 未使能电机发 0xCC 查询
  const auto targets = hold_;
  const auto limits = vel_limit_;
  lock.unlock();

  const auto states = mgr_->snapshot();
  for (size_t i = 0; i < targets.size(); ++i) {
    if (states[i].enabled) {
      mgr_->send_pos_vel(i, targets[i], limits[i]);
    } else {
      mgr_->refresh(i);
    }
  }
}

}  // namespace u1_arm
