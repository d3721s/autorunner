#include "autorunner_driver/driver_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include "autorunner_driver/protocol/frame_ids.hpp"

namespace autorunner_driver
{

using namespace std::chrono_literals;
namespace proto = autorunner::protocol;
using drivers::socketcan::CanId;
using drivers::socketcan::FrameType;
using drivers::socketcan::StandardFrame;

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

  encoder_ = std::make_unique<proto::Encoder>(control_offset);
  decoder_ = std::make_unique<proto::Decoder>(feedback_offset);

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

  // ---- 订阅器 (仅急停/MIT 保持 topic) ----
  stop_sub_ = create_subscription<msgs::Stop>(
    "~/stop_cmd", reliable_qos,
    std::bind(&DriverNode::stop_callback, this, std::placeholders::_1), sub_opts);
  joint_mit_sub_ = create_subscription<msgs::Jointmit>(
    "~/joint_mit_cmd", reliable_qos,
    std::bind(&DriverNode::joint_mit_callback, this, std::placeholders::_1), sub_opts);

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
    this, "~/move_joint",
    std::bind(&DriverNode::handle_goal<FollowJointTrajectory>, this, _1, _2),
    std::bind(&DriverNode::handle_cancel<FollowJointTrajectory>, this, _1),
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> h) {
      std::thread{std::bind(&DriverNode::execute_move_joint, this, _1), h}.detach();
    },
    rcl_action_server_get_default_options(), action_group_);

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
  running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
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
  if (publish_raw_frames_ && raw_rx_pub_) {
    can_msgs::msg::Frame raw;
    raw.header.stamp = now();
    raw.id = id;
    raw.dlc = dlc;
    std::copy(data, data + std::min<size_t>(dlc, 8), raw.data.begin());
    raw_rx_pub_->publish(raw);
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
    // 旁路: 更新 action 运动跟踪并唤醒 execute 线程
    if (motion_active_.load()) {
      motion_status_.store(fb.motion_status);
      motion_arm_status_.store(fb.arm_status);
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
      msgs::Armpose msg;
      msg.x = static_cast<float>((*full)[0]);
      msg.y = static_cast<float>((*full)[1]);
      msg.z = static_cast<float>((*full)[2]);
      msg.rx = static_cast<float>((*full)[3]);
      msg.ry = static_cast<float>((*full)[4]);
      msg.rz = static_cast<float>((*full)[5]);
      arm_position_pub_->publish(msg);
    }
  } else if (std::holds_alternative<proto::JointAnglePart>(decoded)) {
    const auto & part = std::get<proto::JointAnglePart>(decoded);
    if (auto full = joint_angle_assembler_.feed(part.part, part.a, part.b)) {
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        last_joint_angle_ = *full;
      }
      msgs::Jointangle msg;
      for (int i = 0; i < 6; ++i) {
        msg.joint[i] = static_cast<float>((*full)[i]);
      }
      joint_angle_pub_->publish(msg);
      publish_joint_states(*full);
    }
  } else if (std::holds_alternative<proto::DriverHighSpeedFb>(decoded)) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    high_snapshot_.update(std::get<proto::DriverHighSpeedFb>(decoded));
  } else if (std::holds_alternative<proto::DriverLowSpeedFb>(decoded)) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    low_snapshot_.update(std::get<proto::DriverLowSpeedFb>(decoded));
  } else if (std::holds_alternative<proto::JointLimitFb>(decoded)) {
    const auto & fb = std::get<proto::JointLimitFb>(decoded);
    if (fb.joint_num == expected_joint_num_limit_.load()) {
      pending_jointlimit_.notify({fb.max_angle, fb.min_angle, fb.max_speed});
    }
  } else if (std::holds_alternative<proto::JointMaxAccFb>(decoded)) {
    const auto & fb = std::get<proto::JointMaxAccFb>(decoded);
    if (fb.joint_num == expected_joint_num_acc_.load()) {
      pending_jointacc_.notify({fb.max_acc});
    }
  } else if (std::holds_alternative<proto::CollisionLevelFb>(decoded)) {
    const auto & fb = std::get<proto::CollisionLevelFb>(decoded);
    pending_collision_.notify({fb.level});
  } else if (std::holds_alternative<proto::EndVelAccFb>(decoded)) {
    const auto & fb = std::get<proto::EndVelAccFb>(decoded);
    pending_endvelacc_.notify(
      {fb.max_linear_vel, fb.max_angular_vel, fb.max_linear_acc, fb.max_angular_acc});
  } else if (std::holds_alternative<proto::SetResponseFb>(decoded)) {
    const auto & fb = std::get<proto::SetResponseFb>(decoded);
    if (fb.cmd_index == 0x71) {
      pending_enable_.notify({false});
    } else if (fb.cmd_index == 0x75) {
      pending_setjoint_.notify({fb.zero_set_success});
    }
  }
}

bool DriverNode::send_frame(const proto::RawFrame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  try {
    const CanId can_id(frame.id, 0, FrameType::DATA, StandardFrame);
    sender_->send(frame.data.data(), frame.dlc, can_id, 100ms);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "CAN 发送失败 id=0x%X: %s", frame.id, e.what());
    return false;
  }
}

bool DriverNode::send_frames(const std::vector<proto::RawFrame> & frames)
{
  bool ok = true;
  for (const auto & f : frames) {
    ok = send_frame(f) && ok;
  }
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
  proto::MotionCtrlCmd cmd;
  cmd.emergency_stop = msg->state ? 0x01 : 0x02;
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
