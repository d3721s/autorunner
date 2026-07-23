// u1_arm 命令节点: 笛卡尔运动 (MoveL/MoveC/MoveJ_P/offset/movep_canfd)、
// 笛卡尔示教 jog, 以及查询类 (arm_state/original_state、版本、坐标系、realtime_push)。
#include "u1_arm_driver/u1_arm_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "autorunner_ros_interfaces/msg/movejp.hpp"
#include "autorunner_ros_interfaces/msg/movel.hpp"
#include "autorunner_ros_interfaces/msg/moveloffset.hpp"
#include "autorunner_ros_interfaces/msg/u1_movec.hpp"
#include "autorunner_ros_interfaces/msg/cartepos.hpp"
#include "autorunner_ros_interfaces/msg/carteposcustom.hpp"
#include "autorunner_ros_interfaces/msg/armstate.hpp"
#include "autorunner_ros_interfaces/msg/armoriginalstate.hpp"
#include "autorunner_ros_interfaces/msg/armsoftversion.hpp"
#include "autorunner_ros_interfaces/msg/jointversion.hpp"
#include "autorunner_ros_interfaces/msg/toolsoftwareversionv4.hpp"
#include "autorunner_ros_interfaces/msg/robot_info.hpp"
#include "autorunner_ros_interfaces/msg/getallframe.hpp"
#include "autorunner_ros_interfaces/msg/setrealtimepush.hpp"

namespace u1_arm
{

namespace msgs = autorunner_ros_interfaces::msg;

namespace
{
Pose to_pose(const geometry_msgs::msg::Pose & p)
{
  Pose o;
  o.x = p.position.x; o.y = p.position.y; o.z = p.position.z;
  o.qx = p.orientation.x; o.qy = p.orientation.y;
  o.qz = p.orientation.z; o.qw = p.orientation.w;
  return o;
}
}  // namespace

std::string format_pose(const Pose & p, int precision = 4)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision)
      << "{x=" << p.x << ", y=" << p.y << ", z=" << p.z
      << ", qx=" << p.qx << ", qy=" << p.qy
      << ", qz=" << p.qz << ", qw=" << p.qw << "}";
  return out.str();
}

bool U1ArmDriver::run_cartesian(std::vector<std::vector<double>> pts, bool block)
{
  if (pts.empty()) {return false;}
  RCLCPP_INFO(
    get_logger(), "变量下发: cartesian path points=%zu block=%s",
    pts.size(), block ? "true" : "false");
  motors_->set_arm_mode(protocol::CtrlMode::kMit);   // 笛卡尔运动底层走 MIT
  if (!exec_->start_sampled(std::move(pts))) {
    RCLCPP_WARN(get_logger(), "笛卡尔轨迹启动失败");
    return false;
  }
  if (block) {
    const bool done = exec_->wait_motion_done(
      std::chrono::milliseconds(static_cast<int>(kMotionTimeoutS * 1000)));
    RCLCPP_INFO(get_logger(), "笛卡尔轨迹 block 等待完成: done=%s", done ? "true" : "false");
    return done;
  }
  return true;
}

bool U1ArmDriver::run_pos_teach(uint8_t axis, uint8_t dir, uint8_t speed, bool orientation)
{
  if (!kin_) {return false;}
  const auto cur = exec_->commanded();
  const Pose start = kin_->fk(cur);
  // 一次示教走一个固定增量段 (位置 3cm / 姿态 0.15rad), 速度比例缩放段长
  const double ratio = std::clamp(static_cast<double>(speed), 1.0, 100.0) / 100.0;
  const double sign = dir ? 1.0 : -1.0;
  Pose target = start;
  if (!orientation) {
    const double d = 0.03 * ratio * sign;
    if (axis == 0) {target.x += d;} else if (axis == 1) {target.y += d;} else {target.z += d;}
  } else {
    const double a = 0.15 * ratio * sign;
    double roll, pitch, yaw;
    Kinematics::quat_to_euler(start, roll, pitch, yaw);
    if (axis == 0) {roll += a;} else if (axis == 1) {pitch += a;} else {yaw += a;}
    Kinematics::euler_to_quat(roll, pitch, yaw, target);
  }
  auto pts = kin_->cartesian_line(cur, target, 0.002);
  if (!pts) {
    RCLCPP_WARN(get_logger(), "位置/姿态示教轨迹生成失败");
    return false;
  }
  RCLCPP_INFO(
    get_logger(),
    "变量下发: pos_teach axis=%u dir=%u speed=%u orientation=%s start=%s target=%s points=%zu",
    axis, dir, speed, orientation ? "true" : "false", format_pose(start).c_str(),
    format_pose(target).c_str(), pts->size());
  return run_cartesian(std::move(*pts), /*block=*/ false);
}

void U1ArmDriver::setup_query_topics()
{
  auto motion_g = motion_group_;
  auto query_g = query_group_;

  // ---------- MoveL / MoveJP / MoveC / offset (笛卡尔) ----------
  add_bool_cmd<msgs::Movel>(
    "movel", [this](const msgs::Movel::SharedPtr m) {
      if (!kin_) {return false;}
      auto pts = kin_->cartesian_line(exec_->commanded(), to_pose(m->pose), 0.002);
      RCLCPP_INFO(
        get_logger(), "action/topic movel 变量下发: target_pose=%s block=%s",
        format_pose(to_pose(m->pose)).c_str(), m->block ? "true" : "false");
      return pts && run_cartesian(std::move(*pts), m->block);
    }, motion_g);
  add_bool_cmd<msgs::Movejp>(
    "movej_p", [this](const msgs::Movejp::SharedPtr m) {
      if (!kin_) {return false;}
      auto q = kin_->ik(to_pose(m->pose), exec_->commanded());
      RCLCPP_INFO(
        get_logger(), "action/topic movej_p 变量下发: target_pose=%s speed=%u block=%s",
        format_pose(to_pose(m->pose)).c_str(), m->speed, m->block ? "true" : "false");
      return q && on_movej(*q, m->speed, m->block);
    }, motion_g);
  add_bool_cmd<msgs::U1Movec>(
    "movec", [this](const msgs::U1Movec::SharedPtr m) {
      if (!kin_) {return false;}
      auto pts = kin_->cartesian_arc(
        exec_->commanded(), to_pose(m->pose_mid), to_pose(m->pose_end), 0.002, m->loop);
      RCLCPP_INFO(
        get_logger(),
        "action/topic movec 变量下发: mid=%s end=%s loop=%u speed=%u block=%s",
        format_pose(to_pose(m->pose_mid)).c_str(), format_pose(to_pose(m->pose_end)).c_str(),
        m->loop, m->speed, m->block ? "true" : "false");
      return pts && run_cartesian(std::move(*pts), m->block);
    }, motion_g);
  add_bool_cmd<msgs::Moveloffset>(
    "movel_offset", [this](const msgs::Moveloffset::SharedPtr m) {
      if (!kin_) {return false;}
      // 偏移叠加到当前末端位姿
      Pose t = kin_->fk(exec_->commanded());
      t.x += m->pose.position.x; t.y += m->pose.position.y; t.z += m->pose.position.z;
      auto pts = kin_->cartesian_line(exec_->commanded(), t, 0.002);
      RCLCPP_INFO(
        get_logger(), "action/topic movel_offset 变量下发: offset=(%.4f, %.4f, %.4f) block=%s",
        m->pose.position.x, m->pose.position.y, m->pose.position.z,
        m->block ? "true" : "false");
      return pts && run_cartesian(std::move(*pts), m->block);
    }, motion_g);
  // 位姿透传 (movep_canfd): rm 中无 _result, 仅订阅即发即忘
  add_cmd_noresult<msgs::Cartepos>(
    "movep_canfd", [this](const msgs::Cartepos::SharedPtr m) {
      if (!kin_) {return;}
      auto q = kin_->ik(to_pose(m->pose), exec_->commanded());
      if (q) {on_movej_canfd(*q);}
    }, motion_g);
  add_cmd_noresult<msgs::Carteposcustom>(
    "movep_canfd_custom",
    [this](const msgs::Carteposcustom::SharedPtr m) {
      if (!kin_) {return;}
      auto q = kin_->ik(to_pose(m->pose), exec_->commanded());
      if (q) {on_movej_canfd(*q);}
    }, motion_g);

  // ---------- 机械臂状态查询 (非 Bool 结果) ----------
  auto arm_state_pub = create_publisher<msgs::Armstate>(
    "u1_arm/get_current_arm_state_result", rclcpp::ParametersQoS());
  auto arm_orig_pub = create_publisher<msgs::Armoriginalstate>(
    "u1_arm/get_current_arm_original_state_result", rclcpp::ParametersQoS());
  other_results_["get_current_arm_state"] = arm_state_pub;
  other_results_["get_current_arm_original_state"] = arm_orig_pub;
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_current_arm_state_result");
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_current_arm_original_state_result");

  rclcpp::SubscriptionOptions opt;
  opt.callback_group = query_g;
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_current_arm_state_cmd", rclcpp::ParametersQoS(),
      [this, arm_state_pub, arm_orig_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_current_arm_state_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        const auto st = motors_->snapshot();
        const size_t n = st.size();
        std::vector<double> joints(n);
        for (size_t i = 0; i < n; ++i) {
          joints[i] = st[i].joint_pos;
        }

        msgs::Armstate a;
        a.joint.resize(n);
        for (size_t i = 0; i < n; ++i) {
          a.joint[i] = static_cast<float>(joints[i]);
        }
        a.dof = static_cast<uint8_t>(n);
        a.err = 0;
        a.err_len = 0;
        msgs::Armoriginalstate ao;
        ao.joint = a.joint;
        ao.dof = a.dof;
        if (kin_) {
          const auto p = kin_->fk(joints);
          a.pose.position.x = p.x; a.pose.position.y = p.y; a.pose.position.z = p.z;
          a.pose.orientation.x = p.qx; a.pose.orientation.y = p.qy;
          a.pose.orientation.z = p.qz; a.pose.orientation.w = p.qw;
          double r, pi, y;
          Kinematics::quat_to_euler(p, r, pi, y);
          ao.pose = {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
            static_cast<float>(r), static_cast<float>(pi), static_cast<float>(y)};
        } else {
          a.pose.orientation.w = 1.0;
        }
        arm_state_pub->publish(a);
        arm_orig_pub->publish(ao);
        RCLCPP_INFO(
          get_logger(),
          "topic u1_arm/get_current_arm_state_result 发布: dof=%u pose={x=%.4f,y=%.4f,z=%.4f,"
          "qx=%.4f,qy=%.4f,qz=%.4f,qw=%.4f} original_pose=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
          a.dof, a.pose.position.x, a.pose.position.y, a.pose.position.z, a.pose.orientation.x,
          a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w,
          ao.pose[0], ao.pose[1], ao.pose[2], ao.pose[3], ao.pose[4], ao.pose[5]);
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_current_arm_state_cmd");

  // ---------- 版本查询 (静态字符串) ----------
  auto arm_ver_pub = create_publisher<msgs::Armsoftversion>(
    "u1_arm/get_arm_software_version_result", rclcpp::ParametersQoS());
  other_results_["get_arm_software_version"] = arm_ver_pub;
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_arm_software_version_result");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_arm_software_version_cmd", rclcpp::ParametersQoS(),
      [this, arm_ver_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_arm_software_version_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        msgs::Armsoftversion v;
        v.product_version = "U1_ARM";
        v.controller_version = "u1-damiao-1.0";
        v.state = true;
        v.productversion = "U1_ARM";
        arm_ver_pub->publish(v);
        RCLCPP_INFO(
          get_logger(),
          "topic u1_arm/get_arm_software_version_result 发布: product_version=%s "
          "controller_version=%s state=%s",
          v.product_version.c_str(), v.controller_version.c_str(),
          v.state ? "true" : "false");
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_arm_software_version_cmd");

  auto robot_info_pub = create_publisher<msgs::RobotInfo>(
    "u1_arm/get_robot_info_result", rclcpp::ParametersQoS());
  other_results_["get_robot_info"] = robot_info_pub;
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_robot_info_result");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_robot_info_cmd", rclcpp::ParametersQoS(),
      [this, robot_info_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_robot_info_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        msgs::RobotInfo r;
        r.arm_dof = static_cast<uint8_t>(motor_cfgs_.size());
        r.arm_model = 0;
        r.force_type = 0;
        r.robot_controller_version = 1;
        r.state = true;
        robot_info_pub->publish(r);
        RCLCPP_INFO(
          get_logger(),
          "topic u1_arm/get_robot_info_result 发布: arm_dof=%u state=%s",
          r.arm_dof, r.state ? "true" : "false");
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_robot_info_cmd");

  auto joint_ver_pub = create_publisher<msgs::Jointversion>(
    "u1_arm/get_joint_software_version_result", rclcpp::ParametersQoS());
  other_results_["get_joint_software_version"] = joint_ver_pub;
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_joint_software_version_result");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_joint_software_version_cmd", rclcpp::ParametersQoS(),
      [this, joint_ver_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_joint_software_version_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        msgs::Jointversion v;
        v.joint_version.assign(motor_cfgs_.size(), "1.0.0");
        v.state = true;
        joint_ver_pub->publish(v);
        RCLCPP_INFO(
          get_logger(),
          "topic u1_arm/get_joint_software_version_result 发布: joint_count=%zu state=%s",
          v.joint_version.size(), v.state ? "true" : "false");
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_joint_software_version_cmd");

  auto tool_ver_pub = create_publisher<msgs::Toolsoftwareversionv4>(
    "u1_arm/get_tool_software_version_result", rclcpp::ParametersQoS());
  other_results_["get_tool_software_version"] = tool_ver_pub;
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_tool_software_version_result");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_tool_software_version_cmd", rclcpp::ParametersQoS(),
      [this, tool_ver_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_tool_software_version_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        msgs::Toolsoftwareversionv4 v;
        v.tool_version = "n/a";
        v.state = false;
        tool_ver_pub->publish(v);
        RCLCPP_INFO(
          get_logger(),
          "topic u1_arm/get_tool_software_version_result 发布: tool_version=%s state=%s",
          v.tool_version.c_str(), v.state ? "true" : "false");
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_tool_software_version_cmd");

  // ---------- 坐标系 (软件注册表) ----------
  auto work_str_pub = create_publisher<std_msgs::msg::String>(
    "u1_arm/get_curr_workFrame_result", rclcpp::ParametersQoS());
  auto tool_str_pub = create_publisher<std_msgs::msg::String>(
    "u1_arm/get_current_tool_frame_result", rclcpp::ParametersQoS());
  auto all_work_pub = create_publisher<msgs::Getallframe>(
    "u1_arm/get_all_work_frame_result", rclcpp::ParametersQoS());
  auto all_tool_pub = create_publisher<msgs::Getallframe>(
    "u1_arm/get_all_tool_frame_result", rclcpp::ParametersQoS());
  other_results_["get_curr_workFrame"] = work_str_pub;
  other_results_["get_current_tool_frame"] = tool_str_pub;
  other_results_["get_all_work_frame"] = all_work_pub;
  other_results_["get_all_tool_frame"] = all_tool_pub;
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_curr_workFrame_result");
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_current_tool_frame_result");
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_all_work_frame_result");
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_all_tool_frame_result");

  add_bool_cmd<std_msgs::msg::String>(
    "change_work_frame",
    [this](const std_msgs::msg::String::SharedPtr m) {
      RCLCPP_INFO(
        get_logger(), "变量更改请求: cur_work_frame_ %s -> %s",
        cur_work_frame_.c_str(), m->data.c_str());
      cur_work_frame_ = m->data;
      return true;
    },
    query_g);
  add_bool_cmd<std_msgs::msg::String>(
    "change_tool_frame",
    [this](const std_msgs::msg::String::SharedPtr m) {
      RCLCPP_INFO(
        get_logger(), "变量更改请求: cur_tool_frame_ %s -> %s",
        cur_tool_frame_.c_str(), m->data.c_str());
      cur_tool_frame_ = m->data;
      return true;
    },
    query_g);
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_curr_workFrame_cmd", rclcpp::ParametersQoS(),
      [this, work_str_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_curr_workFrame_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        std_msgs::msg::String s; s.data = cur_work_frame_; work_str_pub->publish(s);
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_curr_workFrame_result 发布: data=%s",
          s.data.c_str());
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_curr_workFrame_cmd");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_current_tool_frame_cmd", rclcpp::ParametersQoS(),
      [this, tool_str_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_current_tool_frame_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        std_msgs::msg::String s; s.data = cur_tool_frame_; tool_str_pub->publish(s);
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_current_tool_frame_result 发布: data=%s",
          s.data.c_str());
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_current_tool_frame_cmd");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_all_work_frame_cmd", rclcpp::ParametersQoS(),
      [all_work_pub, this](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_all_work_frame_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        msgs::Getallframe f; f.frame_name[0] = cur_work_frame_; all_work_pub->publish(f);
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_all_work_frame_result 发布: frame_name[0]=%s",
          f.frame_name[0].c_str());
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_all_work_frame_cmd");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_all_tool_frame_cmd", rclcpp::ParametersQoS(),
      [all_tool_pub, this](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_all_tool_frame_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        msgs::Getallframe f; f.frame_name[0] = cur_tool_frame_; all_tool_pub->publish(f);
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_all_tool_frame_result 发布: frame_name[0]=%s",
          f.frame_name[0].c_str());
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_all_tool_frame_cmd");

  // ---------- realtime_push (set=Bool / get=Setrealtimepush) ----------
  add_bool_cmd<msgs::Setrealtimepush>(
    "set_realtime_push",
    [this](const msgs::Setrealtimepush::SharedPtr m) {
      // 记录周期/坐标系 (发布节点周期在启动时确定, 此处仅接受并回 true)
      RCLCPP_INFO(
        get_logger(),
        "变量更改请求: pub_cfg_.force_coordinate %d -> %u, udp_cycle_ms=%d, joint_speed_enable=%s",
        pub_cfg_.force_coordinate, m->force_coordinate, pub_cfg_.udp_cycle_ms,
        pub_cfg_.joint_speed_enable ? "true" : "false");
      pub_cfg_.force_coordinate = m->force_coordinate;
      return true;
    }, query_g);
  auto rtp_pub = create_publisher<msgs::Setrealtimepush>(
    "u1_arm/get_realtime_push_result", rclcpp::ParametersQoS());
  other_results_["get_realtime_push"] = rtp_pub;
  RCLCPP_INFO(get_logger(), "接口创建: publisher u1_arm/get_realtime_push_result");
  subs_.push_back(
    create_subscription<std_msgs::msg::Empty>(
      "u1_arm/get_realtime_push_cmd", rclcpp::ParametersQoS(),
      [this, rtp_pub](const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "topic u1_arm/get_realtime_push_cmd 收到:\n%s",
          message_to_yaml(*msg).c_str());
        msgs::Setrealtimepush v;
        v.cycle = static_cast<uint16_t>(pub_cfg_.udp_cycle_ms);
        v.force_coordinate = static_cast<uint16_t>(pub_cfg_.force_coordinate);
        v.joint_speed_enable = pub_cfg_.joint_speed_enable;
        rtp_pub->publish(v);
        RCLCPP_INFO(
          get_logger(),
          "topic u1_arm/get_realtime_push_result 发布: cycle=%u force_coordinate=%u "
          "joint_speed_enable=%s",
          v.cycle, v.force_coordinate, v.joint_speed_enable ? "true" : "false");
      }, opt));
  RCLCPP_INFO(get_logger(), "接口创建: subscription u1_arm/get_realtime_push_cmd");
}

}  // namespace u1_arm
