// u1_arm 命令节点: 参数加载、电机初始化、200Hz 控制回路、看门狗,
// 以及运动/示教/停止类 u1_arm/*_cmd 回调 (已实现集)。
// 查询类见 u1_arm_node_query.cpp, 打桩类见 u1_arm_node_stubs.cpp。
#include "u1_arm_driver/u1_arm_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_msgs/msg/empty.hpp"

#include "autorunner_ros_interfaces/msg/jointteach.hpp"
#include "autorunner_ros_interfaces/msg/posteach.hpp"
#include "autorunner_ros_interfaces/msg/ortteach.hpp"
#include "autorunner_ros_interfaces/msg/jointerrclear.hpp"
#include "autorunner_ros_interfaces/msg/u1_movej.hpp"
#include "autorunner_ros_interfaces/msg/jointpos.hpp"
#include "autorunner_ros_interfaces/msg/jointposcustom.hpp"
#include "autorunner_ros_interfaces/msg/u1_stop.hpp"
#include "autorunner_ros_interfaces/msg/rmerr.hpp"

namespace u1_arm
{

using namespace std::chrono_literals;
namespace msgs = autorunner_ros_interfaces::msg;

U1ArmDriver::U1ArmDriver(const rclcpp::NodeOptions & options)
: rclcpp::Node("u1_arm_driver", options)
{
  // ---- 参数 ----
  declare_parameter<std::string>("arm_type", "U1_ARM");
  declare_parameter<int>("arm_dof", 5);
  const auto can_interface = declare_parameter<std::string>("can_interface", "can0");
  auto_enable_ = declare_parameter<bool>("auto_enable", true);
  set_mode_on_start_ = declare_parameter<bool>("set_mode_on_start", true);
  estop_disable_motors_ = declare_parameter<bool>("estop_disable_motors", false);
  const int udp_cycle = declare_parameter<int>("udp_cycle", 5);
  // 控制环频率独立于状态上报: 默认 2ms=500Hz (5 电机经典CAN 5000fps, 在预算内)
  const int control_cycle = declare_parameter<int>("control_cycle_ms", 2);
  const auto urdf_path = declare_parameter<std::string>("urdf_path", "");
  const auto base_link = declare_parameter<std::string>("base_link", "l0");
  const auto tip_link = declare_parameter<std::string>("tip_link", "l5");

  motor_cfgs_ = load_motor_configs();

  pub_cfg_.joint_names.clear();
  for (const auto & c : motor_cfgs_) {
    pub_cfg_.joint_names.push_back(c.joint_name);
  }
  pub_cfg_.udp_cycle_ms = udp_cycle;
  pub_cfg_.force_coordinate = declare_parameter<int>("udp_force_coordinate", 0);
  pub_cfg_.joint_speed_enable = declare_parameter<bool>("udp_joint_speed_state", true);

  // ---- CAN 总线 + 电机管理 ----
  try {
    bus_ = std::make_shared<CanBus>(
      can_interface,
      [this](uint32_t id, const uint8_t * d, uint8_t dlc) {
        if (motors_) {motors_->handle_frame(id, d, dlc);}
      });
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "打开 CAN 接口 %s 失败: %s", can_interface.c_str(), e.what());
    throw;
  }
  RCLCPP_INFO(
    get_logger(), "CAN 接口 %s 已打开, %zu 个关节电机", can_interface.c_str(),
    motor_cfgs_.size());

  motors_ = std::make_shared<MotorManager>(motor_cfgs_, bus_);
  const double dt = std::max(1, control_cycle) / 1000.0;
  exec_ = std::make_shared<TrajectoryExecutor>(motors_, dt);

  // ---- 运动学 (可选) ----
  std::string urdf = urdf_path;
  if (urdf.empty()) {
    try {
      urdf = ament_index_cpp::get_package_share_directory("autorunner_driver") +
        "/urdf/u1_arm.urdf";
    } catch (const std::exception &) {
      urdf.clear();
    }
  }
  if (!urdf.empty() && std::filesystem::exists(urdf)) {
    try {
      std::vector<double> lo, hi;
      for (const auto & c : motor_cfgs_) {
        lo.push_back(c.limit_lower); hi.push_back(c.limit_upper);
      }
      kin_ = std::make_shared<Kinematics>(urdf, base_link, tip_link, lo, hi);
      RCLCPP_INFO(
        get_logger(), "运动学链 %s->%s 就绪 (%zu 关节)",
        base_link.c_str(), tip_link.c_str(), kin_->num_joints());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "运动学初始化失败(笛卡尔运动将不可用): %s", e.what());
    }
  } else {
    RCLCPP_WARN(get_logger(), "未找到 URDF, 笛卡尔运动(MoveL/C/JP)将不可用");
  }

  // ---- 回调组 ----
  motion_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  query_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  stub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  setup_motion_topics();
  setup_query_topics();
  setup_stub_topics();

  // ---- 使能 ----
  // 底层默认 MIT 模式: 电机需处于 CTRL_MODE=MIT(1) 才响应 MIT 偏移(0x000)的使能帧。
  // 故先逐个写 CTRL_MODE 寄存器(默认 MIT), 再在当前模式偏移使能。否则模式不符的
  // 电机会静默忽略使能, 表现为"部分轴使能不了"。
  if (auto_enable_) {
    if (set_mode_on_start_) {
      for (size_t i = 0; i < motors_->size(); ++i) {
        motors_->write_ctrl_mode(i);
        std::this_thread::sleep_for(2ms);   // 给电机处理寄存器写的时间
      }
      std::this_thread::sleep_for(10ms);
    }
    motors_->enable_all();
    RCLCPP_INFO(
      get_logger(), "已发送全部电机 MIT 模式设置与使能帧 (控制环 %d ms)", control_cycle);
  }

  // ---- 定时器 ----
  const auto period = std::chrono::milliseconds(std::max(1, control_cycle));
  control_timer_ = create_wall_timer(
    period, std::bind(&U1ArmDriver::control_tick, this), timer_group_);
  watchdog_timer_ = create_wall_timer(
    500ms, std::bind(&U1ArmDriver::watchdog_tick, this), timer_group_);
}

std::vector<MotorConfig> U1ArmDriver::load_motor_configs()
{
  const auto joints = declare_parameter<std::vector<std::string>>(
    "arm_joints", std::vector<std::string>{"j1", "j2", "j3", "j4", "j5"});
  const size_t n = joints.size();

  auto dbl = [&](const std::string & name, std::vector<double> def) {
      auto v = declare_parameter<std::vector<double>>(name, def);
      v.resize(n, def.empty() ? 0.0 : def.back());
      return v;
    };
  auto ints = [&](const std::string & name, std::vector<int64_t> def) {
      auto v = declare_parameter<std::vector<int64_t>>(name, def);
      v.resize(n, def.empty() ? 0 : def.back());
      return v;
    };

  const auto models = declare_parameter<std::vector<std::string>>(
    "motor_models", std::vector<std::string>(n, "DM-J4310"));
  const auto can_ids = ints("motor_can_ids", {1, 2, 3, 4, 5});
  const auto master_ids = ints("motor_master_ids", {0x11, 0x12, 0x13, 0x14, 0x15});
  const auto p_max = dbl("motor_p_max", {12.5});
  const auto v_max = dbl("motor_v_max", {30.0});
  const auto t_max = dbl("motor_t_max", {10.0});
  const auto dir = ints("motor_direction", {1, 1, 1, 1, 1});
  const auto zoff = dbl("motor_zero_offset", {0.0});
  const auto lo = dbl("joint_lower", {-3.14});
  const auto hi = dbl("joint_upper", {3.14});
  const auto vmax = dbl("joint_vmax", {2.0});
  const auto amax = dbl("joint_amax", {5.0});
  const auto kp = dbl("mit_kp", {30.0});   // MIT 位置刚度 (0~500), 需现场整定
  const auto kd = dbl("mit_kd", {1.5});    // MIT 速度阻尼 (0~5), 位置控制时必须 >0

  std::vector<MotorConfig> cfgs(n);
  for (size_t i = 0; i < n; ++i) {
    auto & c = cfgs[i];
    c.joint_name = joints[i];
    c.model = i < models.size() ? models[i] : "DM-J4310";
    c.can_id = static_cast<uint16_t>(can_ids[i]);
    c.master_id = static_cast<uint16_t>(master_ids[i]);
    c.limits = {static_cast<float>(p_max[i]), static_cast<float>(v_max[i]),
      static_cast<float>(t_max[i])};
    c.direction = static_cast<int>(dir[i]) >= 0 ? 1 : -1;
    c.zero_offset = zoff[i];
    c.limit_lower = lo[i];
    c.limit_upper = hi[i];
    c.vmax = vmax[i];
    c.amax = amax[i];
    c.mit_kp = kp[i];
    c.mit_kd = kd[i];
    if (c.mit_kp > 1e-6 && c.mit_kd < 1e-6) {
      RCLCPP_WARN(
        get_logger(), "[%s] mit_kd=0 且 mit_kp>0 会导致 MIT 位置控制震荡, 请设 kd>0",
        c.joint_name.c_str());
    }
  }
  return cfgs;
}

void U1ArmDriver::publish_bool(const std::string & base, bool ok)
{
  auto it = bool_results_.find(base);
  if (it != bool_results_.end()) {
    std_msgs::msg::Bool m;
    m.data = ok;
    it->second->publish(m);
  }
}

void U1ArmDriver::control_tick()
{
  exec_->tick();
}

void U1ArmDriver::watchdog_tick()
{
  const auto stale = motors_->check_timeout(500ms);
  if (!stale.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%zu 个电机反馈超时", stale.size());
    auto it = other_results_.find("udp_rm_err");
    if (it != other_results_.end()) {
      msgs::Rmerr err;
      err.err_len = static_cast<uint8_t>(stale.size());
      for (auto idx : stale) {
        err.err.push_back(static_cast<int32_t>(idx + 1));
      }
      std::static_pointer_cast<rclcpp::Publisher<msgs::Rmerr>>(it->second)->publish(err);
    }
  }

  const auto states = motors_->snapshot();
  ++watchdog_ticks_;

  // 启动诊断: 第 3 次看门狗(~1.5s)一次性打印各电机反馈/使能情况, 便于定位
  // "部分轴使能不了"(常见: can_id/master_id/模式与 yaml 不符)。
  if (watchdog_ticks_ == 3) {
    for (size_t i = 0; i < states.size(); ++i) {
      const auto & c = motor_cfgs_[i];
      if (!states[i].online) {
        RCLCPP_WARN(
          get_logger(),
          "[%s] 无反馈: 未收到 master_id=0x%02X 的帧 —— 检查该电机 CAN ID(0x%02X)/"
          "Master ID/接线/波特率是否与 yaml 一致",
          c.joint_name.c_str(), c.master_id, c.can_id);
      } else if (!states[i].enabled) {
        RCLCPP_WARN(
          get_logger(),
          "[%s] 有反馈但未使能(状态码=%d): 若为 0(失能), 多因电机不在位置速度模式;"
          " 用 motor_setup.py write %d 0x0A 2 && save 后重试",
          c.joint_name.c_str(), static_cast<int>(states[i].status), c.can_id);
      } else {
        RCLCPP_INFO(get_logger(), "[%s] 已使能 ✓", c.joint_name.c_str());
      }
    }
  }

  // 使能重试: 对在线但未使能(且非故障)的电机周期性重发使能, 兼容上电时序/丢帧。
  // 完全无反馈的电机不重试(重试也无意义, 且避免刷屏), 由上面的诊断提示排查。
  if (auto_enable_ && !exec_->estopped()) {
    for (size_t i = 0; i < states.size(); ++i) {
      const auto & s = states[i];
      if (s.online && !s.enabled && !protocol::status_is_fault(s.status)) {
        if (set_mode_on_start_) {motors_->write_ctrl_mode(i);}
        motors_->enable(i);
      }
    }
  }
}

// ======================= 运动回调实现 =======================

bool U1ArmDriver::on_movej(const std::vector<double> & joint, uint8_t speed, bool block)
{
  if (joint.size() != motor_cfgs_.size()) {
    RCLCPP_WARN(get_logger(), "movej 关节数 %zu != %zu", joint.size(), motor_cfgs_.size());
    return false;
  }
  motors_->set_arm_mode(protocol::CtrlMode::kMit);   // 规划类运动走 MIT
  if (!exec_->start_movej(joint, speed)) {return false;}
  if (block) {
    return exec_->wait_motion_done(
      std::chrono::milliseconds(static_cast<int>(kMotionTimeoutS * 1000)));
  }
  return true;
}

bool U1ArmDriver::on_movej_canfd(const std::vector<double> & joint)
{
  if (joint.size() != motor_cfgs_.size()) {return false;}
  motors_->set_arm_mode(protocol::CtrlMode::kPosVel);   // 透传走位置速度模式
  return exec_->passthrough(joint);
}

void U1ArmDriver::setup_motion_topics()
{
  auto g = motion_group_;

  // MoveJ (关节空间梯形/五次插值)
  add_bool_cmd<msgs::U1Movej>(
    "movej", [this](const msgs::U1Movej::SharedPtr m) {
      return on_movej(std::vector<double>(m->joint.begin(), m->joint.end()), m->speed, m->block);
    }, g);

  // 关节透传 (canfd): rm 中无 _result, 仅订阅即发即忘
  add_cmd_noresult<msgs::Jointpos>(
    "movej_canfd", [this](const msgs::Jointpos::SharedPtr m) {
      on_movej_canfd(std::vector<double>(m->joint.begin(), m->joint.end()));
    }, g);
  add_cmd_noresult<msgs::Jointposcustom>(
    "movej_canfd_custom",
    [this](const msgs::Jointposcustom::SharedPtr m) {
      on_movej_canfd(std::vector<double>(m->joint.begin(), m->joint.end()));
    }, g);

  // 停止 / 急停 / 暂停 / 继续
  add_bool_cmd<std_msgs::msg::Empty>(
    "move_stop", [this](const std_msgs::msg::Empty::SharedPtr) {
      exec_->stop();
      return true;
    }, g);
  add_bool_cmd<msgs::U1Stop>(
    "emergency_stop", [this](const msgs::U1Stop::SharedPtr m) {
      if (m->state) {
        exec_->estop();
        if (estop_disable_motors_) {motors_->disable_all();}
      } else {
        if (estop_disable_motors_) {motors_->enable_all();}
        exec_->release_estop();
      }
      return true;
    }, g);
  add_bool_cmd<std_msgs::msg::Empty>(
    "pause", [this](const std_msgs::msg::Empty::SharedPtr) {
      exec_->pause();
      return true;
    }, g);
  add_bool_cmd<std_msgs::msg::Empty>(
    "set_arm_continue",
    [this](const std_msgs::msg::Empty::SharedPtr) {
      exec_->resume();
      return true;
    }, g);

  // 示教: 关节 jog / 笛卡尔位置 jog / 姿态 jog / 停止
  add_bool_cmd<msgs::Jointteach>(
    "set_joint_teach", [this](const msgs::Jointteach::SharedPtr m) {
      if (m->num < 1 || m->num > motor_cfgs_.size()) {return false;}
      motors_->set_arm_mode(protocol::CtrlMode::kMit);   // jog 底层走 MIT
      return exec_->start_jog(m->num - 1, m->direction ? 1 : -1, m->speed);
    }, g);
  add_bool_cmd<msgs::Posteach>(
    "set_pos_teach", [this](const msgs::Posteach::SharedPtr m) {
      return run_pos_teach(m->type, m->direction, m->speed, /*orientation=*/ false);
    }, g);
  add_bool_cmd<msgs::Ortteach>(
    "set_ort_teach", [this](const msgs::Ortteach::SharedPtr m) {
      return run_pos_teach(m->type, m->direction, m->speed, /*orientation=*/ true);
    }, g);
  add_bool_cmd<std_msgs::msg::Empty>(
    "set_stop_teach",
    [this](const std_msgs::msg::Empty::SharedPtr) {
      exec_->stop_jog();
      return true;
    }, g);

  // 清关节错误 (重发使能帧)
  add_bool_cmd<msgs::Jointerrclear>(
    "set_joint_err_clear",
    [this](const msgs::Jointerrclear::SharedPtr m) {
      if (m->joint_num < 1 || m->joint_num > motor_cfgs_.size()) {return false;}
      const size_t i = m->joint_num - 1;
      return motors_->clear_error(i) && motors_->enable(i);
    }, g);
}

}  // namespace u1_arm
