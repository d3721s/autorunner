// u1_arm 状态发布节点: 按 udp_cycle 周期发布 joint_states 与 u1_arm/udp_* 状态话题。
// 数据取自 MotorManager 反馈快照 + Kinematics FK。QoS 统一为 10 (镜像 rm_driver)。
#include "u1_arm_driver/u1_arm_nodes.hpp"

#include <chrono>

#include "autorunner_ros_interfaces/msg/jointenflag.hpp"
#include "autorunner_ros_interfaces/msg/jointposeeuler.hpp"
#include "autorunner_ros_interfaces/msg/u1_jointcurrent.hpp"
#include "autorunner_ros_interfaces/msg/u1_jointerrorcode.hpp"
#include "autorunner_ros_interfaces/msg/u1_jointspeed.hpp"
#include "autorunner_ros_interfaces/msg/u1_jointtemperature.hpp"
#include "autorunner_ros_interfaces/msg/u1_jointvoltage.hpp"

namespace u1_arm
{

namespace msgs = autorunner_ros_interfaces::msg;

U1StatePublisher::U1StatePublisher(
  std::shared_ptr<MotorManager> motors,
  std::shared_ptr<TrajectoryExecutor> exec,
  std::shared_ptr<Kinematics> kin,
  PublishConfig cfg,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("u1_udp_publish_node", options),
  motors_(std::move(motors)), exec_(std::move(exec)), kin_(std::move(kin)), cfg_(std::move(cfg))
{
  const auto qos = rclcpp::QoS(10);
  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  arm_position_pub_ =
    create_publisher<geometry_msgs::msg::Pose>("u1_arm/udp_arm_position", qos);
  joint_speed_pub_ = create_publisher<msgs::U1Jointspeed>("u1_arm/udp_joint_speed", qos);
  joint_temperature_pub_ =
    create_publisher<msgs::U1Jointtemperature>("u1_arm/udp_joint_temperature", qos);
  joint_current_pub_ = create_publisher<msgs::U1Jointcurrent>("u1_arm/udp_joint_current", qos);
  joint_voltage_pub_ = create_publisher<msgs::U1Jointvoltage>("u1_arm/udp_joint_voltage", qos);
  joint_en_flag_pub_ = create_publisher<msgs::Jointenflag>("u1_arm/udp_joint_en_flag", qos);
  joint_error_code_pub_ =
    create_publisher<msgs::U1Jointerrorcode>("u1_arm/udp_joint_error_code", qos);
  joint_pose_euler_pub_ =
    create_publisher<msgs::Jointposeeuler>("u1_arm/udp_joint_pose_euler", qos);

  const auto period = std::chrono::milliseconds(std::max(1, cfg_.udp_cycle_ms));
  timer_ = create_wall_timer(period, std::bind(&U1StatePublisher::publish_tick, this));
  RCLCPP_INFO(get_logger(), "状态发布节点启动, 周期 %d ms", cfg_.udp_cycle_ms);
}

void U1StatePublisher::publish_tick()
{
  const auto states = motors_->snapshot();
  const size_t n = states.size();
  const auto stamp = now();

  std::vector<double> joints(n);
  for (size_t i = 0; i < n; ++i) {
    joints[i] = states[i].joint_pos;
  }

  // joint_states
  sensor_msgs::msg::JointState js;
  js.header.stamp = stamp;
  js.name = cfg_.joint_names;
  js.position.resize(n);
  js.velocity.resize(n);
  js.effort.resize(n);
  for (size_t i = 0; i < n; ++i) {
    js.position[i] = states[i].joint_pos;
    js.velocity[i] = states[i].joint_vel;
    js.effort[i] = states[i].torque;
  }
  joint_states_pub_->publish(js);

  // 关节速度
  msgs::U1Jointspeed spd;
  spd.joint_speed.resize(n);
  for (size_t i = 0; i < n; ++i) {
    spd.joint_speed[i] = static_cast<float>(states[i].joint_vel);
  }
  std::static_pointer_cast<rclcpp::Publisher<msgs::U1Jointspeed>>(joint_speed_pub_)->publish(spd);

  // 关节温度 (线圈温度)
  msgs::U1Jointtemperature temp;
  temp.joint_temperature.resize(n);
  for (size_t i = 0; i < n; ++i) {
    temp.joint_temperature[i] = states[i].t_rotor;
  }
  std::static_pointer_cast<rclcpp::Publisher<msgs::U1Jointtemperature>>(
    joint_temperature_pub_)->publish(temp);

  // 关节电流 (由扭矩粗略换算, 无电流反馈时用扭矩占位)
  msgs::U1Jointcurrent cur;
  cur.joint_current.resize(n);
  for (size_t i = 0; i < n; ++i) {
    cur.joint_current[i] = static_cast<float>(states[i].torque);
  }
  std::static_pointer_cast<rclcpp::Publisher<msgs::U1Jointcurrent>>(
    joint_current_pub_)->publish(cur);

  // 关节电压 (达妙反馈帧无电压字段, 初版恒 0, 后续可用 0x7FF 低速轮询补)
  msgs::U1Jointvoltage volt;
  volt.joint_voltage.assign(n, 0.0f);
  std::static_pointer_cast<rclcpp::Publisher<msgs::U1Jointvoltage>>(
    joint_voltage_pub_)->publish(volt);

  // 使能标志
  msgs::Jointenflag en;
  en.joint_en_flag.resize(n);
  for (size_t i = 0; i < n; ++i) {
    en.joint_en_flag[i] = states[i].enabled;
  }
  std::static_pointer_cast<rclcpp::Publisher<msgs::Jointenflag>>(
    joint_en_flag_pub_)->publish(en);

  // 关节错误码 (电机状态码, 使能/失能视为 0)
  msgs::U1Jointerrorcode err;
  err.joint_error.resize(n);
  err.dof = static_cast<uint8_t>(n);
  for (size_t i = 0; i < n; ++i) {
    const auto s = states[i].status;
    err.joint_error[i] = protocol::status_is_fault(s) ? static_cast<uint16_t>(s) : 0;
  }
  std::static_pointer_cast<rclcpp::Publisher<msgs::U1Jointerrorcode>>(
    joint_error_code_pub_)->publish(err);

  // 末端位姿 (FK) —— 无运动学时发单位位姿
  geometry_msgs::msg::Pose pose;
  msgs::Jointposeeuler pe;
  if (kin_) {
    const auto p = kin_->fk(joints);
    pose.position.x = p.x; pose.position.y = p.y; pose.position.z = p.z;
    pose.orientation.x = p.qx; pose.orientation.y = p.qy;
    pose.orientation.z = p.qz; pose.orientation.w = p.qw;
    double roll, pitch, yaw;
    Kinematics::quat_to_euler(p, roll, pitch, yaw);
    pe.position = {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
    pe.euler = {static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw)};
  } else {
    pose.orientation.w = 1.0;
  }
  arm_position_pub_->publish(pose);
  std::static_pointer_cast<rclcpp::Publisher<msgs::Jointposeeuler>>(
    joint_pose_euler_pub_)->publish(pe);
}

}  // namespace u1_arm
