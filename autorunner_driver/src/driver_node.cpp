#include "autorunner_driver/driver_node.hpp"

#include <chrono>
#include <cmath>
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
  timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cmd_group_;

  const auto reliable_qos = rclcpp::QoS(10);
  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto latched_qos = rclcpp::QoS(1).transient_local();

  // ---- 发布器 ----
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
  joint_limit_pub_ = create_publisher<msgs::Jointlimit>("~/joint_limit", reliable_qos);
  joint_max_acc_pub_ = create_publisher<msgs::Jointmaxacc>("~/joint_max_acc", reliable_qos);
  collision_level_state_pub_ =
    create_publisher<msgs::Collisionlevel>("~/collision_level_state", latched_qos);
  end_velacc_state_pub_ = create_publisher<msgs::Endvelacc>("~/end_velacc_state", latched_qos);
  set_response_pub_ = create_publisher<msgs::Setresponse>("~/set_response", reliable_qos);
  movej_result_pub_ = create_publisher<std_msgs::msg::Bool>("~/movej_result", reliable_qos);
  movep_result_pub_ = create_publisher<std_msgs::msg::Bool>("~/movep_result", reliable_qos);
  movel_result_pub_ = create_publisher<std_msgs::msg::Bool>("~/movel_result", reliable_qos);
  movec_result_pub_ = create_publisher<std_msgs::msg::Bool>("~/movec_result", reliable_qos);
  enable_result_pub_ = create_publisher<std_msgs::msg::Bool>("~/enable_result", reliable_qos);
  if (publish_raw_frames_) {
    raw_rx_pub_ = create_publisher<can_msgs::msg::Frame>("~/raw_rx", sensor_qos);
  }

  // ---- 订阅器 ----
  movej_sub_ = create_subscription<msgs::Movej>(
    "~/movej_cmd", reliable_qos,
    std::bind(&DriverNode::movej_callback, this, std::placeholders::_1), sub_opts);
  movep_sub_ = create_subscription<msgs::Movep>(
    "~/movep_cmd", reliable_qos,
    std::bind(&DriverNode::movep_callback, this, std::placeholders::_1), sub_opts);
  movel_sub_ = create_subscription<msgs::Movep>(
    "~/movel_cmd", reliable_qos,
    std::bind(&DriverNode::movel_callback, this, std::placeholders::_1), sub_opts);
  movec_sub_ = create_subscription<msgs::Movec>(
    "~/movec_cmd", reliable_qos,
    std::bind(&DriverNode::movec_callback, this, std::placeholders::_1), sub_opts);
  stop_sub_ = create_subscription<msgs::Stop>(
    "~/stop_cmd", reliable_qos,
    std::bind(&DriverNode::stop_callback, this, std::placeholders::_1), sub_opts);
  motion_ctrl_sub_ = create_subscription<msgs::Motionctrl>(
    "~/motion_ctrl_cmd", reliable_qos,
    std::bind(&DriverNode::motion_ctrl_callback, this, std::placeholders::_1), sub_opts);
  mode_ctrl_sub_ = create_subscription<msgs::Modectrl>(
    "~/mode_ctrl_cmd", reliable_qos,
    std::bind(&DriverNode::mode_ctrl_callback, this, std::placeholders::_1), sub_opts);
  enable_sub_ = create_subscription<msgs::Jointenable>(
    "~/enable_cmd", reliable_qos,
    std::bind(&DriverNode::enable_callback, this, std::placeholders::_1), sub_opts);
  joint_config_sub_ = create_subscription<msgs::Jointconfig>(
    "~/joint_config_cmd", reliable_qos,
    std::bind(&DriverNode::joint_config_callback, this, std::placeholders::_1), sub_opts);
  joint_limit_query_sub_ = create_subscription<msgs::Jointlimitquery>(
    "~/joint_limit_query_cmd", reliable_qos,
    std::bind(&DriverNode::joint_limit_query_callback, this, std::placeholders::_1), sub_opts);
  joint_limit_set_sub_ = create_subscription<msgs::Jointlimitset>(
    "~/joint_limit_set_cmd", reliable_qos,
    std::bind(&DriverNode::joint_limit_set_callback, this, std::placeholders::_1), sub_opts);
  collision_level_sub_ = create_subscription<msgs::Collisionlevel>(
    "~/collision_level_cmd", reliable_qos,
    std::bind(&DriverNode::collision_level_callback, this, std::placeholders::_1), sub_opts);
  end_velacc_set_sub_ = create_subscription<msgs::Endvelacc>(
    "~/end_velacc_set_cmd", reliable_qos,
    std::bind(&DriverNode::end_velacc_set_callback, this, std::placeholders::_1), sub_opts);
  joint_mit_sub_ = create_subscription<msgs::Jointmit>(
    "~/joint_mit_cmd", reliable_qos,
    std::bind(&DriverNode::joint_mit_callback, this, std::placeholders::_1), sub_opts);

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
      // receive 超时抛 std::runtime_error, 其它异常也不应终止收帧线程
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
    msgs::Jointlimit msg;
    msg.joint_num = fb.joint_num;
    msg.max_angle = static_cast<float>(fb.max_angle);
    msg.min_angle = static_cast<float>(fb.min_angle);
    msg.max_speed = static_cast<float>(fb.max_speed);
    joint_limit_pub_->publish(msg);
  } else if (std::holds_alternative<proto::JointMaxAccFb>(decoded)) {
    const auto & fb = std::get<proto::JointMaxAccFb>(decoded);
    msgs::Jointmaxacc msg;
    msg.joint_num = fb.joint_num;
    msg.max_acc = static_cast<float>(fb.max_acc);
    joint_max_acc_pub_->publish(msg);
  } else if (std::holds_alternative<proto::CollisionLevelFb>(decoded)) {
    const auto & fb = std::get<proto::CollisionLevelFb>(decoded);
    msgs::Collisionlevel msg;
    for (int i = 0; i < 6; ++i) {
      msg.level[i] = fb.level[i];
    }
    collision_level_state_pub_->publish(msg);
  } else if (std::holds_alternative<proto::EndVelAccFb>(decoded)) {
    const auto & fb = std::get<proto::EndVelAccFb>(decoded);
    msgs::Endvelacc msg;
    msg.max_linear_vel = static_cast<float>(fb.max_linear_vel);
    msg.max_angular_vel = static_cast<float>(fb.max_angular_vel);
    msg.max_linear_acc = static_cast<float>(fb.max_linear_acc);
    msg.max_angular_acc = static_cast<float>(fb.max_angular_acc);
    end_velacc_state_pub_->publish(msg);
  } else if (std::holds_alternative<proto::SetResponseFb>(decoded)) {
    const auto & fb = std::get<proto::SetResponseFb>(decoded);
    msgs::Setresponse msg;
    msg.cmd_index = fb.cmd_index;
    msg.zero_set_success = fb.zero_set_success;
    set_response_pub_->publish(msg);
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
  // 协议流程: 0x471 使能全部关节 -> 0x151 进入 CAN 控制模式
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
      RCLCPP_INFO(get_logger(), "初始化序列已发送 (第 %d 次): 使能全部关节 + CAN 控制模式",
        attempt);
      return;
    }
    RCLCPP_WARN(get_logger(), "初始化序列发送失败, 重试 %d/%d", attempt, init_retry_count_);
    std::this_thread::sleep_for(1s);
  }
  RCLCPP_ERROR(get_logger(), "初始化序列发送失败, 已达最大重试次数");
}

// ---------------- 命令回调 ----------------

void DriverNode::movej_callback(const msgs::Movej::SharedPtr msg)
{
  proto::JointTargetCmd target;
  for (int i = 0; i < 6; ++i) {
    target.joint[i] = msg->joint[i];
  }
  proto::ModeCtrlCmd mode;
  mode.ctrl_mode = 0x01;
  mode.move_mode = 0x01;  // MOVE J
  mode.speed = msg->speed > 0 ? msg->speed : default_speed_;
  mode.install_pos = install_pos_;

  const auto frames = encoder_->encode(target);
  bool ok = send_frames({frames.begin(), frames.end()});
  ok = send_frame(encoder_->encode(mode)) && ok;

  std_msgs::msg::Bool result;
  result.data = ok;
  movej_result_pub_->publish(result);
}

void DriverNode::movep_callback(const msgs::Movep::SharedPtr msg)
{
  proto::PoseTargetCmd target;
  target.x = msg->pose.x;
  target.y = msg->pose.y;
  target.z = msg->pose.z;
  target.rx = msg->pose.rx;
  target.ry = msg->pose.ry;
  target.rz = msg->pose.rz;
  proto::ModeCtrlCmd mode;
  mode.ctrl_mode = 0x01;
  mode.move_mode = 0x00;  // MOVE P
  mode.speed = msg->speed > 0 ? msg->speed : default_speed_;
  mode.install_pos = install_pos_;

  const auto frames = encoder_->encode(target);
  bool ok = send_frames({frames.begin(), frames.end()});
  ok = send_frame(encoder_->encode(mode)) && ok;

  std_msgs::msg::Bool result;
  result.data = ok;
  movep_result_pub_->publish(result);
}

void DriverNode::movel_callback(const msgs::Movep::SharedPtr msg)
{
  proto::PoseTargetCmd target;
  target.x = msg->pose.x;
  target.y = msg->pose.y;
  target.z = msg->pose.z;
  target.rx = msg->pose.rx;
  target.ry = msg->pose.ry;
  target.rz = msg->pose.rz;
  proto::ModeCtrlCmd mode;
  mode.ctrl_mode = 0x01;
  mode.move_mode = 0x02;  // MOVE L
  mode.speed = msg->speed > 0 ? msg->speed : default_speed_;
  mode.install_pos = install_pos_;

  const auto frames = encoder_->encode(target);
  bool ok = send_frames({frames.begin(), frames.end()});
  ok = send_frame(encoder_->encode(mode)) && ok;

  std_msgs::msg::Bool result;
  result.data = ok;
  movel_result_pub_->publish(result);
}

void DriverNode::movec_callback(const msgs::Movec::SharedPtr msg)
{
  // 圆弧: 起点(当前位姿)/中点/终点依次发送位姿目标 + 0x158 标记
  auto send_point = [this](const proto::PoseTargetCmd & pose, uint8_t index) {
      const auto frames = encoder_->encode(pose);
      bool ok = send_frames({frames.begin(), frames.end()});
      proto::ArcPointCmd arc;
      arc.point_index = index;
      return send_frame(encoder_->encode(arc)) && ok;
    };

  // 起点: 使用最近一次末端位姿反馈 (0x2A2~4 组装缓存)
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

  proto::PoseTargetCmd mid;
  mid.x = msg->pose_mid.x;
  mid.y = msg->pose_mid.y;
  mid.z = msg->pose_mid.z;
  mid.rx = msg->pose_mid.rx;
  mid.ry = msg->pose_mid.ry;
  mid.rz = msg->pose_mid.rz;
  ok = send_point(mid, 0x02) && ok;

  proto::PoseTargetCmd end;
  end.x = msg->pose_end.x;
  end.y = msg->pose_end.y;
  end.z = msg->pose_end.z;
  end.rx = msg->pose_end.rx;
  end.ry = msg->pose_end.ry;
  end.rz = msg->pose_end.rz;
  ok = send_point(end, 0x03) && ok;

  proto::ModeCtrlCmd mode;
  mode.ctrl_mode = 0x01;
  mode.move_mode = 0x03;  // MOVE C
  mode.speed = msg->speed > 0 ? msg->speed : default_speed_;
  mode.install_pos = install_pos_;
  ok = send_frame(encoder_->encode(mode)) && ok;

  std_msgs::msg::Bool result;
  result.data = ok;
  movec_result_pub_->publish(result);
}

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

void DriverNode::motion_ctrl_callback(const msgs::Motionctrl::SharedPtr msg)
{
  proto::MotionCtrlCmd cmd;
  cmd.trajectory_ctrl = msg->trajectory_ctrl;
  cmd.drag_teach = msg->drag_teach;
  send_frame(encoder_->encode(cmd));
}

void DriverNode::mode_ctrl_callback(const msgs::Modectrl::SharedPtr msg)
{
  proto::ModeCtrlCmd cmd;
  cmd.ctrl_mode = msg->ctrl_mode;
  cmd.move_mode = msg->move_mode;
  cmd.speed = msg->speed;
  cmd.mit_mode = msg->mit_mode;
  cmd.install_pos = msg->install_pos;
  send_frame(encoder_->encode(cmd));
}

void DriverNode::enable_callback(const msgs::Jointenable::SharedPtr msg)
{
  proto::MotorEnableCmd cmd;
  cmd.joint_num = msg->joint_num;
  cmd.enable = msg->enable;
  const bool ok = send_frame(encoder_->encode(cmd));
  std_msgs::msg::Bool result;
  result.data = ok;
  enable_result_pub_->publish(result);
}

void DriverNode::joint_config_callback(const msgs::Jointconfig::SharedPtr msg)
{
  proto::JointConfigCmd cmd;
  cmd.joint_num = msg->joint_num;
  cmd.set_zero = msg->set_zero;
  cmd.clear_err = msg->clear_err;
  cmd.max_acc = msg->max_acc;
  send_frame(encoder_->encode(cmd));
}

void DriverNode::joint_limit_query_callback(const msgs::Jointlimitquery::SharedPtr msg)
{
  proto::JointLimitQueryCmd cmd;
  cmd.joint_num = msg->joint_num;
  cmd.query_type = msg->query_type;
  send_frame(encoder_->encode(cmd));
}

void DriverNode::joint_limit_set_callback(const msgs::Jointlimitset::SharedPtr msg)
{
  proto::JointLimitSetCmd cmd;
  cmd.joint_num = msg->joint_num;
  cmd.max_angle = msg->max_angle;
  cmd.min_angle = msg->min_angle;
  cmd.max_speed = msg->max_speed;
  send_frame(encoder_->encode(cmd));
}

void DriverNode::collision_level_callback(const msgs::Collisionlevel::SharedPtr msg)
{
  proto::CollisionLevelSetCmd cmd;
  for (int i = 0; i < 6; ++i) {
    cmd.level[i] = msg->level[i];
  }
  send_frame(encoder_->encode(cmd));
}

void DriverNode::end_velacc_set_callback(const msgs::Endvelacc::SharedPtr msg)
{
  proto::EndVelAccSetCmd cmd;
  cmd.max_linear_vel = msg->max_linear_vel;
  cmd.max_angular_vel = msg->max_angular_vel;
  cmd.max_linear_acc = msg->max_linear_acc;
  cmd.max_angular_acc = msg->max_angular_acc;
  send_frame(encoder_->encode(cmd));
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
