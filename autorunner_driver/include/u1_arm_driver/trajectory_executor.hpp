// 轨迹执行器: MoveJ 五次多项式同步插值 / 采样轨迹(MoveL/C 由运动学层生成) /
// 恒速 jog 示教 / 透传 / 停止·急停·暂停·继续 语义
//
// tick() 由 200Hz 控制定时器调用: 采样当前 setpoint 并经 MotorManager 下发
// 位置速度帧; 空闲/急停时重发保持位置(达妙收帧即回反馈 -> joint_states 稳定频率);
// 电机未使能时改发 0xCC 查询反馈。
#ifndef U1_ARM_DRIVER__TRAJECTORY_EXECUTOR_HPP_
#define U1_ARM_DRIVER__TRAJECTORY_EXECUTOR_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

#include "u1_arm_driver/motor_manager.hpp"

namespace u1_arm
{

class TrajectoryExecutor
{
public:
  TrajectoryExecutor(std::shared_ptr<MotorManager> mgr, double tick_period_s);

  // ---- 运动命令 (估计被 ROS 回调线程调用; 内部加锁) ----
  // 关节空间点到点: 五次多项式、全关节同步到达。speed 1~100 缩放 vmax/amax。
  // 返回 false: 急停锁存中 / 目标越限 / 电机未就绪。
  bool start_movej(const std::vector<double> & goal, uint8_t speed_percent);
  // 预采样轨迹(每 tick 一点, 由 MoveL/MoveC/示教生成)
  bool start_sampled(std::vector<std::vector<double>> points);
  // 关节 jog: dir=+1/-1, speed 1~100 -> vmax 比例; 持续到 stop_jog/限位
  bool start_jog(size_t joint_index, int dir, uint8_t speed_percent);
  void stop_jog();

  // 透传(movej_canfd): 直接把目标写为 hold, 不做规划。急停中拒绝。
  bool passthrough(const std::vector<double> & joints);

  // move_stop: 中止当前轨迹, 锁存当前 setpoint 保持
  void stop();
  // pause / continue: 仅对轨迹/采样轨迹生效(冻结进度)
  void pause();
  void resume();
  // 急停: 中止 + 锁存, 后续运动命令一律拒绝, 直到 release_estop
  void estop();
  void release_estop();
  bool estopped() const;

  bool busy() const;
  // block=true 语义: 等待当前轨迹自然完成。true=完成, false=被打断/超时
  bool wait_motion_done(std::chrono::milliseconds timeout);

  // 当前指令位置(hold/轨迹 setpoint), 供状态查询与 MoveL 起点用
  std::vector<double> commanded() const;

  // 200Hz 控制主循环入口
  void tick();

private:
  enum class Mode {kIdle, kTrajectory, kSampled, kJog};

  // 五次多项式: s(τ)=10τ³-15τ⁴+6τ⁵
  struct Quintic
  {
    std::vector<double> start;
    std::vector<double> delta;
    double duration{0.0};
    double t{0.0};
  };

  struct Sampled
  {
    std::vector<std::vector<double>> points;
    size_t cursor{0};
  };

  struct Jog
  {
    size_t joint{0};
    int dir{1};
    double vel{0.5};
  };

  // 需要持锁调用
  void hold_from_feedback_locked();
  void finish_motion_locked(bool completed);

  std::shared_ptr<MotorManager> mgr_;
  const double dt_;

  mutable std::mutex mutex_;
  std::condition_variable done_cv_;
  Mode mode_{Mode::kIdle};
  bool estopped_{false};
  bool paused_{false};
  bool hold_valid_{false};
  uint64_t motion_seq_{0};       // 轨迹代号, wait 用于识别"自己那条"是否完成
  bool last_motion_completed_{false};
  std::vector<double> hold_;     // 保持/最新 setpoint (关节侧 rad)
  std::vector<double> vel_limit_;  // 下发的速度限幅
  Quintic quintic_;
  Sampled sampled_;
  Jog jog_;
};

}  // namespace u1_arm

#endif  // U1_ARM_DRIVER__TRAJECTORY_EXECUTOR_HPP_
