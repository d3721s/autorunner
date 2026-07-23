#include "autorunner_driver/driver_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include "autorunner_driver/protocol/frame_ids.hpp"

namespace autorunner_driver
{

using namespace std::chrono_literals;
namespace proto = autorunner::protocol;
using drivers::socketcan::CanId;
using drivers::socketcan::FrameType;
using drivers::socketcan::StandardFrame;

namespace
{

std::string format_array6(const std::array<double, 6> & values, int precision = 4)
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

std::string format_raw_frame(const proto::RawFrame & frame)
{
  std::ostringstream out;
  out << "id=0x" << std::uppercase << std::hex << frame.id << std::dec
      << " dlc=" << static_cast<int>(frame.dlc) << " data=[";
  for (size_t i = 0; i < std::min<size_t>(frame.dlc, frame.data.size()); ++i) {
    if (i > 0) {
      out << " ";
    }
    out << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(frame.data[i]) << std::dec << std::setfill(' ');
  }
  out << "]";
  return out.str();
}

std::string format_bytes(const uint8_t * data, uint8_t dlc)
{
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < std::min<size_t>(dlc, 8); ++i) {
    if (i > 0) {
      out << " ";
    }
    out << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(data[i]) << std::dec << std::setfill(' ');
  }
  out << "]";
  return out.str();
}

void log_interface_created(
  const rclcpp::Logger & logger, const char * kind, const std::string & name)
{
  RCLCPP_INFO(logger, "接口创建: %s %s", kind, name.c_str());
}

}  // namespace

DriverNode::DriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("autorunner_driver", options)
{
  // ---- 参数 ----
  const auto can_interface = declare_parameter<std::string>("can_interface", "can0");
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names",
    std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  const bool auto_enable = declare_parameter<bool>("auto_enable", true);
  install_pos_ = static_cast<uint8_t>(declare_parameter<int>("install_pos", 1));
  default_speed_ = static_cast<uint8_t>(declare_parameter<int>("default_speed_percent", 20));
  const double high_rate = declare_parameter<double>("high_speed_publish_rate", 100.0);
  const double low_rate = declare_parameter<double>("low_speed_publish_rate", 10.0);
  publish_raw_frames_ = declare_parameter<bool>("publish_raw_frames", false);
  const auto feedback_offset =
    static_cast<uint32_t>(declare_parameter<int>("feedback_id_offset", 0));
  const auto control_offset =
    static_cast<uint32_t>(declare_parameter<int>("control_id_offset", 0));
  init_retry_count_ = static_cast<int>(declare_parameter<int>("init_retry_count", 3));
  response_timeout_ =
    std::chrono::milliseconds(declare_parameter<int>("response_timeout_ms", 1000));
  motion_timeout_ =
    std::chrono::milliseconds(declare_parameter<int>("motion_timeout_ms", 30000));
  min_move_time_ =
    std::chrono::milliseconds(declare_parameter<int>("min_move_time_ms", 300));
  const auto move_joint_action_name = declare_parameter<std::string>(
    "move_joint_action_name", "/autorunnerbase_controller/follow_joint_trajectory");
  traj_min_interval_ =
    std::chrono::milliseconds(declare_parameter<int>("traj_min_interval_ms", 20));
  encoder_ = std::make_unique<proto::Encoder>(control_offset);
  decoder_ = std::make_unique<proto::Decoder>(feedback_offset);
  RCLCPP_INFO(
    get_logger(),
    "参数加载完成: can_interface=%s joint_count=%zu auto_enable=%s install_pos=%u "
    "default_speed=%u high_rate=%.3f low_rate=%.3f publish_raw_frames=%s "
    "feedback_id_offset=0x%X control_id_offset=0x%X init_retry_count=%d "
    "response_timeout_ms=%ld motion_timeout_ms=%ld min_move_time_ms=%ld "
    "move_joint_action=%s traj_min_interval_ms=%ld",
    can_interface.c_str(), joint_names_.size(), auto_enable ? "true" : "false",
    install_pos_, default_speed_, high_rate, low_rate,
    publish_raw_frames_ ? "true" : "false", feedback_offset, control_offset,
    init_retry_count_, response_timeout_.count(), motion_timeout_.count(),
    min_move_time_.count(), move_joint_action_name.c_str(), traj_min_interval_.count());

  // ---- SocketCAN ----
  try {
    receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface, false);
    sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface, false);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "打开 CAN 接口 %s 失败: %s", can_interface.c_str(), e.what());
    throw;
  }
  RCLCPP_INFO(get_logger(), "CAN 接口 %s 已打开", can_interface.c_str());

  // ---- 回调组 ----
  cmd_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cmd_group_;

  const auto reliable_qos = rclcpp::QoS(10);
  const auto sensor_qos = rclcpp::SensorDataQoS();

  // ---- 发布器 (周期反馈保持 topic) ----
  joint_states_pub_ =
    create_publisher<sensor_msgs::msg::JointState>("/joint_states", reliable_qos);
  arm_status_pub_ = create_publisher<msgs::Armstatus>("~/arm_status", reliable_qos);
  arm_position_pub_ = create_publisher<msgs::Armpose>("~/arm_position", reliable_qos);
  joint_angle_pub_ = create_publisher<msgs::Jointangle>("~/joint_angle", reliable_qos);
  joint_speed_pub_ = create_publisher<msgs::Jointspeed>("~/joint_speed", sensor_qos);
  joint_current_pub_ = create_publisher<msgs::Jointcurrent>("~/joint_current", sensor_qos);
  joint_motor_pos_pub_ = create_publisher<msgs::Jointmotorpos>("~/joint_motor_pos", sensor_qos);
  joint_voltage_pub_ = create_publisher<msgs::Jointvoltage>("~/joint_voltage", reliable_qos);
  joint_temperature_pub_ =
    create_publisher<msgs::Jointtemperature>("~/joint_temperature", reliable_qos);
  joint_error_code_pub_ =
    create_publisher<msgs::Jointerrorcode>("~/joint_error_code", reliable_qos);
  if (publish_raw_frames_) {
    raw_rx_pub_ = create_publisher<can_msgs::msg::Frame>("~/raw_rx", sensor_qos);
  }
  log_interface_created(get_logger(), "publisher", "/joint_states");
  log_interface_created(get_logger(), "publisher", "~/arm_status");
  log_interface_created(get_logger(), "publisher", "~/arm_position");
  log_interface_created(get_logger(), "publisher", "~/joint_angle");
  log_interface_created(get_logger(), "publisher", "~/joint_speed");
  log_interface_created(get_logger(), "publisher", "~/joint_current");
  log_interface_created(get_logger(), "publisher", "~/joint_motor_pos");
  log_interface_created(get_logger(), "publisher", "~/joint_voltage");
  log_interface_created(get_logger(), "publisher", "~/joint_temperature");
  log_interface_created(get_logger(), "publisher", "~/joint_error_code");
  if (publish_raw_frames_) {
    log_interface_created(get_logger(), "publisher", "~/raw_rx");
  }

  // ---- 订阅器 (仅急停/MIT 保持 topic) ----
  stop_sub_ = create_subscription<msgs::Stop>(
    "~/stop_cmd", reliable_qos,
    std::bind(&DriverNode::stop_callback, this, std::placeholders::_1), sub_opts);
  joint_mit_sub_ = create_subscription<msgs::Jointmit>(
    "~/joint_mit_cmd", reliable_qos,
    std::bind(&DriverNode::joint_mit_callback, this, std::placeholders::_1), sub_opts);
  log_interface_created(get_logger(), "subscription", "~/stop_cmd");
  log_interface_created(get_logger(), "subscription", "~/joint_mit_cmd");

  // ---- Service ----
  const auto srv_grp = service_group_;
  enable_joint_srv_ = create_service<srvs::EnableJoint>(
    "~/enable_joint",
    std::bind(
      &DriverNode::enable_joint_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  set_joint_zero_srv_ = create_service<srvs::SetJointZero>(
    "~/set_joint_zero",
    std::bind(
      &DriverNode::set_joint_zero_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  clear_joint_error_srv_ = create_service<srvs::ClearJointError>(
    "~/clear_joint_error",
    std::bind(
      &DriverNode::clear_joint_error_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  set_joint_acc_srv_ = create_service<srvs::SetJointAcc>(
    "~/set_joint_acc",
    std::bind(
      &DriverNode::set_joint_acc_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  query_joint_limit_srv_ = create_service<srvs::QueryJointLimit>(
    "~/query_joint_limit",
    std::bind(
      &DriverNode::query_joint_limit_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  query_joint_max_acc_srv_ = create_service<srvs::QueryJointMaxAcc>(
    "~/query_joint_max_acc",
    std::bind(
      &DriverNode::query_joint_max_acc_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  query_end_vel_acc_srv_ = create_service<srvs::QueryEndVelAcc>(
    "~/query_end_vel_acc",
    std::bind(
      &DriverNode::query_end_vel_acc_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  query_collision_level_srv_ = create_service<srvs::QueryCollisionLevel>(
    "~/query_collision_level",
    std::bind(
      &DriverNode::query_collision_level_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  set_joint_limit_srv_ = create_service<srvs::SetJointLimit>(
    "~/set_joint_limit",
    std::bind(
      &DriverNode::set_joint_limit_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  set_end_vel_acc_srv_ = create_service<srvs::SetEndVelAcc>(
    "~/set_end_vel_acc",
    std::bind(
      &DriverNode::set_end_vel_acc_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  set_collision_level_srv_ = create_service<srvs::SetCollisionLevel>(
    "~/set_collision_level",
    std::bind(
      &DriverNode::set_collision_level_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  set_motion_ctrl_srv_ = create_service<srvs::SetMotionCtrl>(
    "~/set_motion_ctrl",
    std::bind(
      &DriverNode::set_motion_ctrl_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  emergency_stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/emergency_stop",
    std::bind(
      &DriverNode::emergency_stop_service, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_grp);
  log_interface_created(get_logger(), "service", "~/enable_joint");
  log_interface_created(get_logger(), "service", "~/set_joint_zero");
  log_interface_created(get_logger(), "service", "~/clear_joint_error");
  log_interface_created(get_logger(), "service", "~/set_joint_acc");
  log_interface_created(get_logger(), "service", "~/query_joint_limit");
  log_interface_created(get_logger(), "service", "~/query_joint_max_acc");
  log_interface_created(get_logger(), "service", "~/query_end_vel_acc");
  log_interface_created(get_logger(), "service", "~/query_collision_level");
  log_interface_created(get_logger(), "service", "~/set_joint_limit");
  log_interface_created(get_logger(), "service", "~/set_end_vel_acc");
  log_interface_created(get_logger(), "service", "~/set_collision_level");
  log_interface_created(get_logger(), "service", "~/set_motion_ctrl");
  log_interface_created(get_logger(), "service", "~/emergency_stop");

  // ---- Action ----
  using namespace std::placeholders;
  move_p_server_ = rclcpp_action::create_server<acts::MoveP>(
    this, "~/move_p",
    std::bind(&DriverNode::handle_goal<acts::MoveP>, this, _1, _2),
    std::bind(&DriverNode::handle_cancel<acts::MoveP>, this, _1),
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveP>> h) {
      std::thread{std::bind(&DriverNode::execute_move_p, this, _1), h}.detach();
    },
    rcl_action_server_get_default_options(), action_group_);
  move_l_server_ = rclcpp_action::create_server<acts::MoveL>(
    this, "~/move_l",
    std::bind(&DriverNode::handle_goal<acts::MoveL>, this, _1, _2),
    std::bind(&DriverNode::handle_cancel<acts::MoveL>, this, _1),
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveL>> h) {
      std::thread{std::bind(&DriverNode::execute_move_l, this, _1), h}.detach();
    },
    rcl_action_server_get_default_options(), action_group_);
  move_c_server_ = rclcpp_action::create_server<acts::MoveC>(
    this, "~/move_c",
    std::bind(&DriverNode::handle_goal<acts::MoveC>, this, _1, _2),
    std::bind(&DriverNode::handle_cancel<acts::MoveC>, this, _1),
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveC>> h) {
      std::thread{std::bind(&DriverNode::execute_move_c, this, _1), h}.detach();
    },
    rcl_action_server_get_default_options(), action_group_);
  move_joint_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this, move_joint_action_name,
    std::bind(&DriverNode::handle_goal<FollowJointTrajectory>, this, _1, _2),
    std::bind(&DriverNode::handle_cancel<FollowJointTrajectory>, this, _1),
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> h) {
      std::thread{std::bind(&DriverNode::execute_move_joint, this, _1), h}.detach();
    },
    rcl_action_server_get_default_options(), action_group_);
  log_interface_created(get_logger(), "action_server", "~/move_p");
  log_interface_created(get_logger(), "action_server", "~/move_l");
  log_interface_created(get_logger(), "action_server", "~/move_c");
  log_interface_created(get_logger(), "action_server", move_joint_action_name);

  // ---- 定时器 ----
  high_speed_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / high_rate),
    std::bind(&DriverNode::high_speed_timer_callback, this), timer_group_);
  low_speed_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / low_rate),
    std::bind(&DriverNode::low_speed_timer_callback, this), timer_group_);
  watchdog_timer_ = create_wall_timer(
    500ms, std::bind(&DriverNode::watchdog_callback, this), timer_group_);

  // ---- 收帧线程 ----
  running_ = true;
  receive_thread_ = std::thread(&DriverNode::receive_loop, this);
  RCLCPP_INFO(get_logger(), "CAN 接收线程已启动");

  // ---- 自动初始化 (启动 500ms 后执行一次) ----
  if (auto_enable) {
    init_timer_ = create_wall_timer(
      500ms, [this]() {
        init_timer_->cancel();
        initialize_arm();
      }, cmd_group_);
  }
}

DriverNode::~DriverNode()
{
  RCLCPP_INFO(get_logger(), "停止 autorunner_driver 节点, 等待 CAN 接收线程退出");
  running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  RCLCPP_INFO(get_logger(), "CAN 接收线程已退出");
}

// ---------------- CAN 收发 ----------------

void DriverNode::receive_loop()
{
  std::array<uint8_t, 8> buf{};
  while (running_) {
    try {
      buf.fill(0);
      const auto can_id = receiver_->receive(buf.data(), 100ms);
      if (can_id.frame_type() != FrameType::DATA || can_id.is_extended()) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "忽略非标准数据 CAN 帧: id=0x%X extended=%s",
          can_id.identifier(), can_id.is_extended() ? "true" : "false");
        continue;
      }
      dispatch(can_id.identifier(), buf.data(), static_cast<uint8_t>(can_id.length()));
    } catch (const drivers::socketcan::SocketCanTimeout &) {
      // 超时用于响应退出标志, 正常情况
    } catch (const std::exception & e) {
      const std::string what = e.what();
      if (what.find("timeout") == std::string::npos &&
        what.find("Timeout") == std::string::npos)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "CAN 接收异常: %s", what.c_str());
      }
    }
  }
}

void DriverNode::dispatch(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "CAN 收到: id=0x%X dlc=%u data=%s",
    id, dlc, format_bytes(data, dlc).c_str());
  if (publish_raw_frames_ && raw_rx_pub_) {
    can_msgs::msg::Frame raw;
    raw.header.stamp = now();
    raw.id = id;
    raw.dlc = dlc;
    std::copy(data, data + std::min<size_t>(dlc, 8), raw.data.begin());
    raw_rx_pub_->publish(raw);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000, "topic ~/raw_rx 发布: id=0x%X dlc=%u", id, dlc);
  }

  const auto decoded = decoder_->decode(id, data, dlc);

  if (std::holds_alternative<proto::ArmStatusFb>(decoded)) {
    const auto & fb = std::get<proto::ArmStatusFb>(decoded);
    last_status_time_ns_ = now().nanoseconds();
    msgs::Armstatus msg;
    msg.ctrl_mode = fb.ctrl_mode;
    msg.arm_status = fb.arm_status;
    msg.move_mode = fb.move_mode;
    msg.teach_status = fb.teach_status;
    msg.motion_status = fb.motion_status;
    msg.trajectory_num = fb.trajectory_num;
    msg.joint_comm_err = fb.joint_comm_err;
    msg.joint_angle_limit_err = fb.joint_angle_limit_err;
    arm_status_pub_->publish(msg);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "topic ~/arm_status 发布: ctrl_mode=%u arm_status=%u move_mode=%u teach_status=%u "
      "motion_status=%u trajectory_num=%u joint_comm_err=0x%02X joint_angle_limit_err=0x%02X",
      fb.ctrl_mode, fb.arm_status, fb.move_mode, fb.teach_status, fb.motion_status,
      fb.trajectory_num, fb.joint_comm_err, fb.joint_angle_limit_err);
    // 旁路: 更新 action 运动跟踪并唤醒 execute 线程
    if (motion_active_.load()) {
      const auto old_motion = motion_status_.exchange(fb.motion_status);
      const auto old_arm = motion_arm_status_.exchange(fb.arm_status);
      if (old_motion != fb.motion_status || old_arm != fb.arm_status) {
        RCLCPP_INFO(
          get_logger(),
          "运动跟踪变量更新: motion_status %u -> %u, motion_arm_status %u -> %u",
          old_motion, fb.motion_status, old_arm, fb.arm_status);
      }
      motion_cv_.notify_all();
    }
    if (fb.arm_status != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "机械臂状态异常: %u", fb.arm_status);
    }
  } else if (std::holds_alternative<proto::EndPosePart>(decoded)) {
    const auto & part = std::get<proto::EndPosePart>(decoded);
    if (auto full = end_pose_assembler_.feed(part.part, part.a, part.b)) {
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        last_end_pose_ = *full;
      }
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000, "变量更新: last_end_pose=%s",
        format_array6(*full).c_str());
      msgs::Armpose msg;
      msg.x = static_cast<float>((*full)[0]);
      msg.y = static_cast<float>((*full)[1]);
      msg.z = static_cast<float>((*full)[2]);
      msg.rx = static_cast<float>((*full)[3]);
      msg.ry = static_cast<float>((*full)[4]);
      msg.rz = static_cast<float>((*full)[5]);
      arm_position_pub_->publish(msg);
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "topic ~/arm_position 发布: x=%.4f y=%.4f z=%.4f rx=%.4f ry=%.4f rz=%.4f",
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz);
    }
  } else if (std::holds_alternative<proto::JointAnglePart>(decoded)) {
    const auto & part = std::get<proto::JointAnglePart>(decoded);
    if (auto full = joint_angle_assembler_.feed(part.part, part.a, part.b)) {
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        last_joint_angle_ = *full;
      }
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000, "变量更新: last_joint_angle=%s",
        format_array6(*full).c_str());
      msgs::Jointangle msg;
      for (int i = 0; i < 6; ++i) {
        msg.joint[i] = static_cast<float>((*full)[i]);
      }
      joint_angle_pub_->publish(msg);
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000, "topic ~/joint_angle 发布: joint=%s",
        format_array6(*full).c_str());
      publish_joint_states(*full);
    }
  } else if (std::holds_alternative<proto::DriverHighSpeedFb>(decoded)) {
    const auto & fb = std::get<proto::DriverHighSpeedFb>(decoded);
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    high_snapshot_.update(fb);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "变量更新: high_snapshot joint=%u speed=%.4f current=%.4f position=%.4f",
      fb.joint_index, fb.speed, fb.current, fb.position);
  } else if (std::holds_alternative<proto::DriverLowSpeedFb>(decoded)) {
    const auto & fb = std::get<proto::DriverLowSpeedFb>(decoded);
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    low_snapshot_.update(fb);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "变量更新: low_snapshot joint=%u voltage=%.3f bus_current=%.3f driver_temp=%.1f "
      "motor_temp=%.1f status=0x%02X",
      fb.joint_index, fb.voltage, fb.bus_current, fb.driver_temp, fb.motor_temp, fb.status);
  } else if (std::holds_alternative<proto::JointLimitFb>(decoded)) {
    const auto & fb = std::get<proto::JointLimitFb>(decoded);
    if (fb.joint_num == expected_joint_num_limit_.load()) {
      pending_jointlimit_.notify({fb.max_angle, fb.min_angle, fb.max_speed});
      RCLCPP_INFO(
        get_logger(),
        "service 应答匹配 0x473: joint=%u max_angle=%.4f min_angle=%.4f max_speed=%.4f",
        fb.joint_num, fb.max_angle, fb.min_angle, fb.max_speed);
    }
  } else if (std::holds_alternative<proto::JointMaxAccFb>(decoded)) {
    const auto & fb = std::get<proto::JointMaxAccFb>(decoded);
    if (fb.joint_num == expected_joint_num_acc_.load()) {
      pending_jointacc_.notify({fb.max_acc});
      RCLCPP_INFO(
        get_logger(), "service 应答匹配 0x47C: joint=%u max_acc=%.4f",
        fb.joint_num, fb.max_acc);
    }
  } else if (std::holds_alternative<proto::CollisionLevelFb>(decoded)) {
    const auto & fb = std::get<proto::CollisionLevelFb>(decoded);
    pending_collision_.notify({fb.level});
    RCLCPP_INFO(
      get_logger(), "service 应答匹配 0x47B: level=[%u, %u, %u, %u, %u, %u]",
      fb.level[0], fb.level[1], fb.level[2], fb.level[3], fb.level[4], fb.level[5]);
  } else if (std::holds_alternative<proto::EndVelAccFb>(decoded)) {
    const auto & fb = std::get<proto::EndVelAccFb>(decoded);
    pending_endvelacc_.notify(
      {fb.max_linear_vel, fb.max_angular_vel, fb.max_linear_acc, fb.max_angular_acc});
    RCLCPP_INFO(
      get_logger(),
      "service 应答匹配 0x478: max_linear_vel=%.4f max_angular_vel=%.4f "
      "max_linear_acc=%.4f max_angular_acc=%.4f",
      fb.max_linear_vel, fb.max_angular_vel, fb.max_linear_acc, fb.max_angular_acc);
  } else if (std::holds_alternative<proto::SetResponseFb>(decoded)) {
    const auto & fb = std::get<proto::SetResponseFb>(decoded);
    if (fb.cmd_index == 0x71) {
      pending_enable_.notify({false});
      RCLCPP_INFO(get_logger(), "service 应答匹配 0x476: cmd_index=0x71 enable");
    } else if (fb.cmd_index == 0x75) {
      pending_setjoint_.notify({fb.zero_set_success});
      RCLCPP_INFO(
        get_logger(), "service 应答匹配 0x476: cmd_index=0x75 zero_set_success=%s",
        fb.zero_set_success ? "true" : "false");
    }
  }
}

bool DriverNode::send_frame(const proto::RawFrame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  try {
    const CanId can_id(frame.id, 0, FrameType::DATA, StandardFrame);
    sender_->send(frame.data.data(), frame.dlc, can_id, 100ms);
    RCLCPP_DEBUG(get_logger(), "CAN 下发成功: %s", format_raw_frame(frame).c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "CAN 发送失败 id=0x%X: %s", frame.id, e.what());
    return false;
  }
}

bool DriverNode::send_frames(const std::vector<proto::RawFrame> & frames)
{
  bool ok = true;
  RCLCPP_DEBUG(get_logger(), "CAN 批量下发开始: frame_count=%zu", frames.size());
  for (const auto & f : frames) {
    ok = send_frame(f) && ok;
  }
  RCLCPP_DEBUG(
    get_logger(), "CAN 批量下发完成: frame_count=%zu ok=%s",
    frames.size(), ok ? "true" : "false");
  return ok;
}

// ---------------- 初始化序列 ----------------

void DriverNode::initialize_arm()
{
  proto::MotorEnableCmd enable_cmd;
  enable_cmd.joint_num = 7;
  enable_cmd.enable = true;

  proto::ModeCtrlCmd mode_cmd;
  mode_cmd.ctrl_mode = 0x01;
  mode_cmd.install_pos = install_pos_;

  for (int attempt = 1; attempt <= init_retry_count_; ++attempt) {
    RCLCPP_INFO(
      get_logger(), "初始化序列下发尝试 %d/%d: enable_all=true ctrl_mode=CAN install_pos=%u",
      attempt, init_retry_count_, install_pos_);
    const bool sent = send_frame(encoder_->encode(enable_cmd)) &&
      send_frame(encoder_->encode(mode_cmd));
    if (sent) {
      RCLCPP_INFO(
        get_logger(), "初始化序列已发送 (第 %d 次): 使能全部关节 + CAN 控制模式",
        attempt);
      return;
    }
    RCLCPP_WARN(get_logger(), "初始化序列发送失败, 重试 %d/%d", attempt, init_retry_count_);
    std::this_thread::sleep_for(1s);
  }
  RCLCPP_ERROR(get_logger(), "初始化序列发送失败, 已达最大重试次数");
}

// ---------------- Topic 回调 (急停/MIT) ----------------

void DriverNode::stop_callback(const msgs::Stop::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "topic ~/stop_cmd 收到: state=%s", msg->state ? "true" : "false");
  proto::MotionCtrlCmd cmd;
  cmd.emergency_stop = msg->state ? 0x01 : 0x02;
  RCLCPP_INFO(
    get_logger(), "变量下发: MotionCtrlCmd.emergency_stop=%u", cmd.emergency_stop);
  send_frame(encoder_->encode(cmd));
  if (msg->state) {
    RCLCPP_WARN(get_logger(), "已发送快速急停");
  } else {
    RCLCPP_INFO(get_logger(), "已发送急停恢复 (需重新使能电机后方可运动)");
  }
}

void DriverNode::joint_mit_callback(const msgs::Jointmit::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "topic ~/joint_mit_cmd 收到");
  RCLCPP_WARN_ONCE(
    get_logger(),
    "MIT 控制 (0x15A~0x15F) 尚未实现: 协议未给出定点量化范围, 待对照 Piper SDK 确认");
}

// ---------------- 反馈发布 ----------------

void DriverNode::publish_joint_states(const std::array<double, 6> & joint)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = joint_names_;
  msg.position.assign(joint.begin(), joint.end());
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    msg.velocity.assign(high_snapshot_.speed.begin(), high_snapshot_.speed.end());
  }
  joint_states_pub_->publish(msg);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "topic /joint_states 发布: position=%s velocity_count=%zu",
    format_array6(joint).c_str(), msg.velocity.size());
}

void DriverNode::high_speed_timer_callback()
{
  msgs::Jointspeed speed_msg;
  msgs::Jointcurrent current_msg;
  msgs::Jointmotorpos pos_msg;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    for (int i = 0; i < 6; ++i) {
      speed_msg.joint_speed[i] = static_cast<float>(high_snapshot_.speed[i]);
      current_msg.joint_current[i] = static_cast<float>(high_snapshot_.current[i]);
      pos_msg.joint_pos[i] = static_cast<float>(high_snapshot_.position[i]);
    }
  }
  joint_speed_pub_->publish(speed_msg);
  joint_current_pub_->publish(current_msg);
  joint_motor_pos_pub_->publish(pos_msg);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "topic ~/joint_speed ~/joint_current ~/joint_motor_pos 发布: "
    "speed=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] current=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] "
    "position=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
    speed_msg.joint_speed[0], speed_msg.joint_speed[1], speed_msg.joint_speed[2],
    speed_msg.joint_speed[3], speed_msg.joint_speed[4], speed_msg.joint_speed[5],
    current_msg.joint_current[0], current_msg.joint_current[1], current_msg.joint_current[2],
    current_msg.joint_current[3], current_msg.joint_current[4], current_msg.joint_current[5],
    pos_msg.joint_pos[0], pos_msg.joint_pos[1], pos_msg.joint_pos[2],
    pos_msg.joint_pos[3], pos_msg.joint_pos[4], pos_msg.joint_pos[5]);
}

void DriverNode::low_speed_timer_callback()
{
  msgs::Jointvoltage voltage_msg;
  msgs::Jointtemperature temp_msg;
  msgs::Jointerrorcode err_msg;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    for (int i = 0; i < 6; ++i) {
      voltage_msg.voltage[i] = static_cast<float>(low_snapshot_.voltage[i]);
      voltage_msg.bus_current[i] = static_cast<float>(low_snapshot_.bus_current[i]);
      temp_msg.driver_temp[i] = static_cast<float>(low_snapshot_.driver_temp[i]);
      temp_msg.motor_temp[i] = static_cast<float>(low_snapshot_.motor_temp[i]);
      err_msg.joint_error[i] = low_snapshot_.status[i];
    }
  }
  joint_voltage_pub_->publish(voltage_msg);
  joint_temperature_pub_->publish(temp_msg);
  joint_error_code_pub_->publish(err_msg);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "topic ~/joint_voltage ~/joint_temperature ~/joint_error_code 发布: "
    "voltage=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] "
    "driver_temp=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] "
    "motor_temp=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] "
    "error=[%u, %u, %u, %u, %u, %u]",
    voltage_msg.voltage[0], voltage_msg.voltage[1], voltage_msg.voltage[2],
    voltage_msg.voltage[3], voltage_msg.voltage[4], voltage_msg.voltage[5],
    temp_msg.driver_temp[0], temp_msg.driver_temp[1], temp_msg.driver_temp[2],
    temp_msg.driver_temp[3], temp_msg.driver_temp[4], temp_msg.driver_temp[5],
    temp_msg.motor_temp[0], temp_msg.motor_temp[1], temp_msg.motor_temp[2],
    temp_msg.motor_temp[3], temp_msg.motor_temp[4], temp_msg.motor_temp[5],
    err_msg.joint_error[0], err_msg.joint_error[1], err_msg.joint_error[2],
    err_msg.joint_error[3], err_msg.joint_error[4], err_msg.joint_error[5]);
}

void DriverNode::watchdog_callback()
{
  const int64_t last = last_status_time_ns_.load();
  if (last == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "尚未收到机械臂状态反馈 (0x2A1), 请检查 CAN 连接");
    return;
  }
  const auto age_ns = now().nanoseconds() - last;
  if (age_ns > 500 * 1000 * 1000LL) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "机械臂状态反馈超时 %.1fs, 请检查 CAN 总线", age_ns / 1e9);
  }
}

}  // namespace autorunner_driver
