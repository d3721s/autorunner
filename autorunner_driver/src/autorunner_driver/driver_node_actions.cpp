// DriverNode 的 action 实现: MoveP/MoveL/MoveC + MoveJ(FollowJointTrajectory)
// 运动完成判定基于 0x2A1 状态反馈 (byte4 运动状态, byte1 机械臂状态), 支持取消。
#include "autorunner_driver/driver_node.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace autorunner_driver
{

using namespace std::chrono_literals;
namespace proto = autorunner::protocol;

namespace
{

std::string format_uuid(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < uuid.size(); ++i) {
    out << std::setw(2) << static_cast<int>(uuid[i]);
  }
  return out.str();
}

std::string format_pose(const proto::PoseTargetCmd & pose)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(4)
      << "{x=" << pose.x << ", y=" << pose.y << ", z=" << pose.z
      << ", rx=" << pose.rx << ", ry=" << pose.ry << ", rz=" << pose.rz << "}";
  return out.str();
}

template<typename ContainerT>
std::string format_values(const ContainerT & values, int precision = 4)
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

const char * motion_phase_name(MotionPhase phase)
{
  switch (phase) {
    case MotionPhase::kIdle:
      return "Idle";
    case MotionPhase::kWaitStart:
      return "WaitStart";
    case MotionPhase::kWaitArrive:
      return "WaitArrive";
  }
  return "Unknown";
}

}  // namespace

// ---- goal/cancel 通用处理 (所有 action 共用) ----
template<typename ActionT>
rclcpp_action::GoalResponse DriverNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const typename ActionT::Goal> goal)
{
  RCLCPP_INFO(
    get_logger(), "action goal request: uuid=%s goal_type=%s motion_active=%s",
    format_uuid(uuid).c_str(), rosidl_generator_traits::name<typename ActionT::Goal>(),
    motion_active_.load() ? "true" : "false");
  (void)goal;
  if (motion_active_.load()) {
    RCLCPP_WARN(get_logger(), "action goal rejected: uuid=%s 已有运动在执行", format_uuid(uuid).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "action goal accepted: uuid=%s", format_uuid(uuid).c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template<typename ActionT>
rclcpp_action::CancelResponse DriverNode::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
{
  (void)handle;
  RCLCPP_INFO(
    get_logger(), "action cancel request: goal_type=%s, 将下发急停",
    rosidl_generator_traits::name<typename ActionT::Goal>());
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 显式实例化
template rclcpp_action::GoalResponse DriverNode::handle_goal<acts::MoveP>(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const acts::MoveP::Goal>);
template rclcpp_action::GoalResponse DriverNode::handle_goal<acts::MoveL>(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const acts::MoveL::Goal>);
template rclcpp_action::GoalResponse DriverNode::handle_goal<acts::MoveC>(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const acts::MoveC::Goal>);
template rclcpp_action::GoalResponse DriverNode::handle_goal<FollowJointTrajectory>(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal>);
template rclcpp_action::CancelResponse DriverNode::handle_cancel<acts::MoveP>(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveP>>);
template rclcpp_action::CancelResponse DriverNode::handle_cancel<acts::MoveL>(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveL>>);
template rclcpp_action::CancelResponse DriverNode::handle_cancel<acts::MoveC>(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveC>>);
template rclcpp_action::CancelResponse DriverNode::handle_cancel<FollowJointTrajectory>(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>);

// ---- 运动状态机等待循环 ----
// 状态: kWaitStart (等 byte4=1 确认开始, 或最小运动时间到) -> kWaitArrive (等 byte4=0 到达)
// 返回 true=正常到达; false=中止/取消(canceled 出参区分); result_arm_status 填结束时机械臂状态
bool DriverNode::wait_motion_done(
  std::function<bool()> is_canceling,
  std::function<void()> publish_feedback,
  uint8_t & result_arm_status,
  bool & canceled)
{
  canceled = false;
  const auto start = std::chrono::steady_clock::now();
  MotionPhase phase = MotionPhase::kWaitStart;
  motion_status_.store(1);   // 复位: 假定尚未到达
  motion_arm_status_.store(0);
  motion_active_.store(true);
  RCLCPP_INFO(
    get_logger(),
    "运动状态机启动: motion_status_=1 motion_arm_status_=0 motion_active_=true phase=%s "
    "timeout_ms=%ld min_move_time_ms=%ld",
    motion_phase_name(phase), motion_timeout_.count(), min_move_time_.count());

  bool arrived = false;
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lock(motion_mutex_);
    motion_cv_.wait_for(lock, 200ms);
    lock.unlock();

    const auto elapsed = std::chrono::steady_clock::now() - start;
    const uint8_t arm_st = motion_arm_status_.load();
    const uint8_t mot_st = motion_status_.load();

    publish_feedback();

    // 取消: 下发急停并结束
    if (is_canceling()) {
      proto::MotionCtrlCmd stop;
      stop.emergency_stop = 0x01;
      RCLCPP_WARN(get_logger(), "运动状态机取消: 下发 emergency_stop=1");
      send_frame(encoder_->encode(stop));
      canceled = true;
      result_arm_status = arm_st;
      break;
    }
    // 机械臂状态异常 (无解/奇异/超限/碰撞/急停等) -> 中止
    if (arm_st != 0) {
      result_arm_status = arm_st;
      RCLCPP_WARN(
        get_logger(), "运动状态机中止: arm_status=%u motion_status=%u phase=%s",
        arm_st, mot_st, motion_phase_name(phase));
      break;
    }
    // 超时 -> 中止
    if (elapsed > motion_timeout_) {
      result_arm_status = arm_st;
      RCLCPP_WARN(get_logger(), "运动超时 %ld ms", motion_timeout_.count());
      break;
    }

    if (phase == MotionPhase::kWaitStart) {
      // 观察到"未到达"确认已开始, 或最小运动时间已过 (防止上次到达残留导致秒完成)
      if (mot_st == 1 || elapsed > min_move_time_) {
        RCLCPP_INFO(
          get_logger(), "运动状态机阶段切换: %s -> %s elapsed_ms=%ld motion_status=%u",
          motion_phase_name(phase), motion_phase_name(MotionPhase::kWaitArrive),
          std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), mot_st);
        phase = MotionPhase::kWaitArrive;
      }
    } else {  // kWaitArrive
      if (mot_st == 0) {
        arrived = true;
        result_arm_status = 0;
        RCLCPP_INFO(
          get_logger(), "运动状态机到达: elapsed_ms=%ld arm_status=%u motion_status=%u",
          std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), arm_st, mot_st);
        break;
      }
    }
  }

  motion_active_.store(false);
  RCLCPP_INFO(
    get_logger(),
    "运动状态机结束: arrived=%s canceled=%s result_arm_status=%u motion_active_=false",
    arrived ? "true" : "false", canceled ? "true" : "false", result_arm_status);
  return arrived;
}

// ---- 笛卡尔运动共用 (MoveP/MoveL) ----
template<typename ActionT>
void DriverNode::run_cartesian_motion(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
  const proto::PoseTargetCmd & target, uint8_t move_mode, uint8_t speed)
{
  auto result = std::make_shared<typename ActionT::Result>();
  RCLCPP_INFO(
    get_logger(), "action 笛卡尔运动下发: target=%s move_mode=0x%02X speed=%u",
    format_pose(target).c_str(), move_mode, speed);

  const auto frames = encoder_->encode(target);
  bool ok = send_frames({frames.begin(), frames.end()});

  proto::ModeCtrlCmd mode;
  mode.ctrl_mode = 0x01;
  mode.move_mode = move_mode;
  mode.speed = speed > 0 ? speed : default_speed_;
  mode.install_pos = install_pos_;
  RCLCPP_INFO(
    get_logger(),
    "变量下发: ModeCtrlCmd{ctrl_mode=0x%02X, move_mode=0x%02X, speed=%u, install_pos=%u}",
    mode.ctrl_mode, mode.move_mode, mode.speed, mode.install_pos);
  ok = send_frame(encoder_->encode(mode)) && ok;

  if (!ok) {
    result->success = false;
    result->message = "CAN 帧下发失败";
    RCLCPP_ERROR(get_logger(), "action 笛卡尔运动 abort: CAN 帧下发失败");
    handle->abort(result);
    return;
  }

  auto publish_fb = [this, handle]() {
      auto fb = std::make_shared<typename ActionT::Feedback>();
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        fb->current_pose.x = static_cast<float>(last_end_pose_[0]);
        fb->current_pose.y = static_cast<float>(last_end_pose_[1]);
        fb->current_pose.z = static_cast<float>(last_end_pose_[2]);
        fb->current_pose.rx = static_cast<float>(last_end_pose_[3]);
        fb->current_pose.ry = static_cast<float>(last_end_pose_[4]);
        fb->current_pose.rz = static_cast<float>(last_end_pose_[5]);
        for (int i = 0; i < 6; ++i) {
          fb->current_joint[i] = static_cast<float>(last_joint_angle_[i]);
        }
      }
      fb->motion_status = motion_status_.load();
      handle->publish_feedback(fb);
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "action 笛卡尔 feedback 发布: pose={x=%.4f,y=%.4f,z=%.4f,rx=%.4f,ry=%.4f,rz=%.4f} "
        "motion_status=%u",
        fb->current_pose.x, fb->current_pose.y, fb->current_pose.z,
        fb->current_pose.rx, fb->current_pose.ry, fb->current_pose.rz, fb->motion_status);
    };

  uint8_t arm_status = 0;
  bool canceled = false;
  const bool arrived = wait_motion_done(
    [handle]() {return handle->is_canceling();}, publish_fb, arm_status, canceled);

  result->arm_status = arm_status;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    result->final_pose.x = static_cast<float>(last_end_pose_[0]);
    result->final_pose.y = static_cast<float>(last_end_pose_[1]);
    result->final_pose.z = static_cast<float>(last_end_pose_[2]);
    result->final_pose.rx = static_cast<float>(last_end_pose_[3]);
    result->final_pose.ry = static_cast<float>(last_end_pose_[4]);
    result->final_pose.rz = static_cast<float>(last_end_pose_[5]);
  }

  if (canceled) {
    result->success = false;
    result->message = "已取消, 已下发急停";
    RCLCPP_WARN(
      get_logger(), "action 笛卡尔 canceled: arm_status=%u final_pose=%s",
      arm_status,
      format_pose(
        {result->final_pose.x, result->final_pose.y, result->final_pose.z,
          result->final_pose.rx, result->final_pose.ry, result->final_pose.rz}).c_str());
    handle->canceled(result);
  } else if (arrived) {
    result->success = true;
    result->message = "已到达目标点位";
    RCLCPP_INFO(
      get_logger(), "action 笛卡尔 succeed: arm_status=%u final_pose=%s",
      arm_status,
      format_pose(
        {result->final_pose.x, result->final_pose.y, result->final_pose.z,
          result->final_pose.rx, result->final_pose.ry, result->final_pose.rz}).c_str());
    handle->succeed(result);
  } else {
    result->success = false;
    result->message = "运动失败 (机械臂状态: " + std::to_string(arm_status) + ")";
    RCLCPP_ERROR(get_logger(), "action 笛卡尔 abort: arm_status=%u", arm_status);
    handle->abort(result);
  }
}

template void DriverNode::run_cartesian_motion<acts::MoveP>(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveP>>,
  const proto::PoseTargetCmd &, uint8_t, uint8_t);
template void DriverNode::run_cartesian_motion<acts::MoveL>(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveL>>,
  const proto::PoseTargetCmd &, uint8_t, uint8_t);

// ---- MoveP (move_mode=0x00) ----
void DriverNode::execute_move_p(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveP>> handle)
{
  const auto goal = handle->get_goal();
  proto::PoseTargetCmd target;
  target.x = goal->target.x;
  target.y = goal->target.y;
  target.z = goal->target.z;
  target.rx = goal->target.rx;
  target.ry = goal->target.ry;
  target.rz = goal->target.rz;
  RCLCPP_INFO(
    get_logger(), "action ~/move_p execute: target=%s speed=%u",
    format_pose(target).c_str(), goal->speed);
  run_cartesian_motion<acts::MoveP>(handle, target, 0x00, goal->speed);
}

// ---- MoveL (move_mode=0x02) ----
void DriverNode::execute_move_l(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveL>> handle)
{
  const auto goal = handle->get_goal();
  proto::PoseTargetCmd target;
  target.x = goal->target.x;
  target.y = goal->target.y;
  target.z = goal->target.z;
  target.rx = goal->target.rx;
  target.ry = goal->target.ry;
  target.rz = goal->target.rz;
  RCLCPP_INFO(
    get_logger(), "action ~/move_l execute: target=%s speed=%u",
    format_pose(target).c_str(), goal->speed);
  run_cartesian_motion<acts::MoveL>(handle, target, 0x02, goal->speed);
}

// ---- MoveC (圆弧, move_mode=0x03): 起点(当前位姿)/中点/终点各带 0x158 标记 ----
void DriverNode::execute_move_c(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveC>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<acts::MoveC::Result>();
  RCLCPP_INFO(
    get_logger(),
    "action ~/move_c execute: mid={x=%.4f,y=%.4f,z=%.4f,rx=%.4f,ry=%.4f,rz=%.4f} "
    "end={x=%.4f,y=%.4f,z=%.4f,rx=%.4f,ry=%.4f,rz=%.4f} speed=%u",
    goal->mid.x, goal->mid.y, goal->mid.z, goal->mid.rx, goal->mid.ry, goal->mid.rz,
    goal->end.x, goal->end.y, goal->end.z, goal->end.rx, goal->end.ry, goal->end.rz,
    goal->speed);

  auto send_point = [this](const proto::PoseTargetCmd & pose, uint8_t index) {
      const auto frames = encoder_->encode(pose);
      bool ok = send_frames({frames.begin(), frames.end()});
      proto::ArcPointCmd arc;
      arc.point_index = index;
      RCLCPP_INFO(
        get_logger(), "变量下发: ArcPoint index=%u pose=%s",
        arc.point_index, format_pose(pose).c_str());
      return send_frame(encoder_->encode(arc)) && ok;
    };

  proto::PoseTargetCmd start;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    start.x = last_end_pose_[0];
    start.y = last_end_pose_[1];
    start.z = last_end_pose_[2];
    start.rx = last_end_pose_[3];
    start.ry = last_end_pose_[4];
    start.rz = last_end_pose_[5];
  }
  bool ok = send_point(start, 0x01);
  RCLCPP_INFO(get_logger(), "action ~/move_c 起点取当前位姿: %s", format_pose(start).c_str());

  proto::PoseTargetCmd mid;
  mid.x = goal->mid.x; mid.y = goal->mid.y; mid.z = goal->mid.z;
  mid.rx = goal->mid.rx; mid.ry = goal->mid.ry; mid.rz = goal->mid.rz;
  ok = send_point(mid, 0x02) && ok;

  proto::PoseTargetCmd end;
  end.x = goal->end.x; end.y = goal->end.y; end.z = goal->end.z;
  end.rx = goal->end.rx; end.ry = goal->end.ry; end.rz = goal->end.rz;
  ok = send_point(end, 0x03) && ok;

  proto::ModeCtrlCmd mode;
  mode.ctrl_mode = 0x01;
  mode.move_mode = 0x03;
  mode.speed = goal->speed > 0 ? goal->speed : default_speed_;
  mode.install_pos = install_pos_;
  RCLCPP_INFO(
    get_logger(),
    "变量下发: ModeCtrlCmd{ctrl_mode=0x%02X, move_mode=0x%02X, speed=%u, install_pos=%u}",
    mode.ctrl_mode, mode.move_mode, mode.speed, mode.install_pos);
  ok = send_frame(encoder_->encode(mode)) && ok;

  if (!ok) {
    result->success = false;
    result->message = "CAN 帧下发失败";
    RCLCPP_ERROR(get_logger(), "action ~/move_c abort: CAN 帧下发失败");
    handle->abort(result);
    return;
  }

  auto publish_fb = [this, handle]() {
      auto fb = std::make_shared<acts::MoveC::Feedback>();
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        fb->current_pose.x = static_cast<float>(last_end_pose_[0]);
        fb->current_pose.y = static_cast<float>(last_end_pose_[1]);
        fb->current_pose.z = static_cast<float>(last_end_pose_[2]);
        fb->current_pose.rx = static_cast<float>(last_end_pose_[3]);
        fb->current_pose.ry = static_cast<float>(last_end_pose_[4]);
        fb->current_pose.rz = static_cast<float>(last_end_pose_[5]);
        for (int i = 0; i < 6; ++i) {
          fb->current_joint[i] = static_cast<float>(last_joint_angle_[i]);
        }
      }
      fb->motion_status = motion_status_.load();
      handle->publish_feedback(fb);
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "action ~/move_c feedback 发布: pose={x=%.4f,y=%.4f,z=%.4f,rx=%.4f,ry=%.4f,rz=%.4f} "
        "motion_status=%u",
        fb->current_pose.x, fb->current_pose.y, fb->current_pose.z,
        fb->current_pose.rx, fb->current_pose.ry, fb->current_pose.rz, fb->motion_status);
    };

  uint8_t arm_status = 0;
  bool canceled = false;
  const bool arrived = wait_motion_done(
    [handle]() {return handle->is_canceling();}, publish_fb, arm_status, canceled);

  result->arm_status = arm_status;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    result->final_pose.x = static_cast<float>(last_end_pose_[0]);
    result->final_pose.y = static_cast<float>(last_end_pose_[1]);
    result->final_pose.z = static_cast<float>(last_end_pose_[2]);
    result->final_pose.rx = static_cast<float>(last_end_pose_[3]);
    result->final_pose.ry = static_cast<float>(last_end_pose_[4]);
    result->final_pose.rz = static_cast<float>(last_end_pose_[5]);
  }

  if (canceled) {
    result->success = false;
    result->message = "已取消, 已下发急停";
    RCLCPP_WARN(get_logger(), "action ~/move_c canceled: arm_status=%u", arm_status);
    handle->canceled(result);
  } else if (arrived) {
    result->success = true;
    result->message = "已到达终点";
    RCLCPP_INFO(get_logger(), "action ~/move_c succeed: arm_status=%u", arm_status);
    handle->succeed(result);
  } else {
    result->success = false;
    result->message = "圆弧运动失败 (机械臂状态: " + std::to_string(arm_status) + ")";
    RCLCPP_ERROR(get_logger(), "action ~/move_c abort: arm_status=%u", arm_status);
    handle->abort(result);
  }
}

// ---- MoveJ: 标准 FollowJointTrajectory (move_mode=0x04 MOVE M, 逐点流式跟随) ----
// 流式逐点下发: 按每点 time_from_start 定时发目标角, 让机械臂沿 MoveIt 规划路径跟随;
// 相邻点最小间隔 traj_min_interval_ 降采样防止 CAN 过载; 末点必发以保证精确到达。
void DriverNode::execute_move_joint(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();

  const auto & traj = goal->trajectory;
  RCLCPP_INFO(
    get_logger(),
    "action FollowJointTrajectory execute: joint_count=%zu point_count=%zu "
    "path_tolerance=%zu goal_tolerance=%zu",
    traj.joint_names.size(), traj.points.size(), goal->path_tolerance.size(),
    goal->goal_tolerance.size());
  if (traj.points.empty()) {
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = "轨迹为空";
    RCLCPP_ERROR(get_logger(), "action FollowJointTrajectory abort: 轨迹为空");
    handle->abort(result);
    return;
  }

  // 关节名映射: MoveIt 的 joint_names 顺序未必是 joint1~6, 按名字映射到本驱动关节序
  // idx_map[k] = 轨迹里 joint_names_[k] 对应的下标; -1 表示缺失
  std::array<int, 6> idx_map;
  idx_map.fill(-1);
  for (size_t k = 0; k < joint_names_.size() && k < 6; ++k) {
    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
      if (traj.joint_names[j] == joint_names_[k]) {
        idx_map[k] = static_cast<int>(j);
        break;
      }
    }
  }
  for (int k = 0; k < 6; ++k) {
    if (idx_map[k] < 0) {
      result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
      result->error_string = "轨迹缺少关节: " + joint_names_[k];
      RCLCPP_ERROR(
        get_logger(), "action FollowJointTrajectory abort: missing joint=%s",
        joint_names_[k].c_str());
      handle->abort(result);
      return;
    }
  }
  RCLCPP_INFO(
    get_logger(), "变量更新: FollowJointTrajectory idx_map=[%d, %d, %d, %d, %d, %d]",
    idx_map[0], idx_map[1], idx_map[2], idx_map[3], idx_map[4], idx_map[5]);

  // 从轨迹点取 6 关节目标 (按 idx_map 重排)
  auto point_to_target = [&idx_map](const trajectory_msgs::msg::JointTrajectoryPoint & p,
      proto::JointTargetCmd & out) -> bool {
      for (int k = 0; k < 6; ++k) {
        if (static_cast<size_t>(idx_map[k]) >= p.positions.size()) {return false;}
        out.joint[k] = p.positions[idx_map[k]];
      }
      return true;
    };

  // 进入 CAN + MOVE M 模式 (仅一次)
  // 流式逐点跟随用 MOVE M (0x04, 多点/连续轨迹), 而非 MOVE J (单目标点内部插值)
  proto::ModeCtrlCmd mode;
  mode.ctrl_mode = 0x01;
  mode.move_mode = 0x04;
  mode.speed = default_speed_;
  mode.install_pos = install_pos_;
  RCLCPP_INFO(
    get_logger(),
    "变量下发: ModeCtrlCmd{ctrl_mode=0x%02X, move_mode=0x%02X, speed=%u, install_pos=%u}",
    mode.ctrl_mode, mode.move_mode, mode.speed, mode.install_pos);
  if (!send_frame(encoder_->encode(mode))) {
    result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
    result->error_string = "CAN 帧下发失败 (模式切换)";
    RCLCPP_ERROR(get_logger(), "action FollowJointTrajectory abort: 模式切换 CAN 下发失败");
    handle->abort(result);
    return;
  }

  auto publish_fb = [this, handle, &traj]() {
      auto fb = std::make_shared<FollowJointTrajectory::Feedback>();
      fb->header.stamp = now();
      fb->joint_names = traj.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint actual;
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        actual.positions.assign(last_joint_angle_.begin(), last_joint_angle_.end());
      }
      fb->actual = actual;
      handle->publish_feedback(fb);
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "action FollowJointTrajectory feedback 发布: actual=%s",
        format_values(actual.positions).c_str());
    };

  // ---- 流式下发阶段 ----
  // 置运动跟踪激活, 使 dispatch 在收到 0x2A1 时更新 motion_arm_status_, 供异常检查
  motion_arm_status_.store(0);
  motion_active_.store(true);
  RCLCPP_INFO(
    get_logger(),
    "变量更新: motion_arm_status_=0 motion_active_=true (FollowJointTrajectory 流式阶段)");
  const auto traj_start = std::chrono::steady_clock::now();
  const size_t n = traj.points.size();

  // 每点相对起始的计划时刻 (ns)
  auto point_time_ns = [](const trajectory_msgs::msg::JointTrajectoryPoint & p) -> int64_t {
      return static_cast<int64_t>(p.time_from_start.sec) * 1000000000LL +
             p.time_from_start.nanosec;
    };
  const int64_t min_interval_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(traj_min_interval_).count();

  int64_t last_scheduled_ns = -min_interval_ns;  // 保证首点必发

  for (size_t i = 0; i < n; ++i) {
    const auto & pt = traj.points[i];
    const bool is_last = (i + 1 == n);
    const int64_t sched_ns = point_time_ns(pt);

    // 降采样: 按计划时刻降采样, 距上次已下发点不足最小间隔则跳过 (但末点必发)
    if (!is_last && (sched_ns - last_scheduled_ns) < min_interval_ns) {
      continue;
    }

    // 取消检查
    if (handle->is_canceling()) {
      proto::MotionCtrlCmd stop;
      stop.emergency_stop = 0x01;
      RCLCPP_WARN(
        get_logger(), "action FollowJointTrajectory canceled at point=%zu: 下发 emergency_stop=1",
        i);
      send_frame(encoder_->encode(stop));
      motion_active_.store(false);
      RCLCPP_INFO(get_logger(), "变量更新: motion_active_=false");
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "已取消, 已下发急停";
      handle->canceled(result);
      return;
    }
    // 机械臂异常检查
    const uint8_t arm_st = motion_arm_status_.load();
    if (arm_st != 0) {
      result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string = "运动中机械臂异常 (状态: " + std::to_string(arm_st) + ")";
      RCLCPP_ERROR(
        get_logger(), "action FollowJointTrajectory abort at point=%zu: arm_status=%u",
        i, arm_st);
      handle->abort(result);
      motion_active_.store(false);
      RCLCPP_INFO(get_logger(), "变量更新: motion_active_=false");
      return;
    }

    // 按 time_from_start 节拍: 等到该点的计划时刻再下发
    std::this_thread::sleep_until(traj_start + std::chrono::nanoseconds(sched_ns));

    proto::JointTargetCmd target;
    if (!point_to_target(pt, target)) {
      RCLCPP_WARN(get_logger(), "action FollowJointTrajectory 跳过轨迹点 %zu: 关节数据不全", i);
      continue;  // 该点关节数据不全, 跳过
    }
    RCLCPP_DEBUG(
      get_logger(), "变量下发: JointTargetCmd point=%zu target=%s",
      i, format_values(target.joint).c_str());
    const auto frames = encoder_->encode(target);
    if (!send_frames({frames.begin(), frames.end()})) {
      motion_active_.store(false);
      RCLCPP_INFO(get_logger(), "变量更新: motion_active_=false");
      result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string = "CAN 帧下发失败 (轨迹点 " + std::to_string(i) + ")";
      RCLCPP_ERROR(get_logger(), "action FollowJointTrajectory abort: 轨迹点 %zu CAN 下发失败", i);
      handle->abort(result);
      return;
    }
    // MOVE M 模式帧在流式过程中周期性补发, 确保机械臂处于运动模式
    send_frame(encoder_->encode(mode));
    last_scheduled_ns = sched_ns;
    publish_fb();
  }

  // ---- 末点已发, 等待到达终点 ----
  uint8_t arm_status = 0;
  bool canceled = false;
  const bool arrived = wait_motion_done(
    [handle]() {return handle->is_canceling();}, publish_fb, arm_status, canceled);

  if (canceled) {
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    result->error_string = "已取消, 已下发急停";
    RCLCPP_WARN(get_logger(), "action FollowJointTrajectory canceled: arm_status=%u", arm_status);
    handle->canceled(result);
  } else if (arrived) {
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    result->error_string = "已跟随轨迹到达终点";
    RCLCPP_INFO(get_logger(), "action FollowJointTrajectory succeed: arm_status=%u", arm_status);
    handle->succeed(result);
  } else {
    result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
    result->error_string = "运动失败 (机械臂状态: " + std::to_string(arm_status) + ")";
    RCLCPP_ERROR(get_logger(), "action FollowJointTrajectory abort: arm_status=%u", arm_status);
    handle->abort(result);
  }
}

}  // namespace autorunner_driver
