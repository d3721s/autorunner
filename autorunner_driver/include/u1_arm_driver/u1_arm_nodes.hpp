// u1_arm 双节点结构 (镜像 rm_driver):
//   - U1ArmDriver      : 命令节点, 名 "u1_arm_driver", 处理全部 u1_arm/*_cmd
//                        并发布 u1_arm/*_result; 拥有 CanBus/MotorManager/
//                        TrajectoryExecutor/Kinematics 与 200Hz 控制定时器
//   - U1StatePublisher : 状态节点, 名 "u1_udp_publish_node", 按 udp_cycle 发布
//                        joint_states 与 u1_arm/udp_* 状态话题
// 话题前缀用 "u1_arm/" (对应 rm_driver 的 "rm_driver/"), joint_states 无前缀。
// cmd/result QoS = ParametersQoS(), 状态话题 QoS = 10 —— 与 rm 完全一致。
#ifndef U1_ARM_DRIVER__U1_ARM_NODES_HPP_
#define U1_ARM_DRIVER__U1_ARM_NODES_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include "u1_arm_driver/can_bus.hpp"
#include "u1_arm_driver/kinematics.hpp"
#include "u1_arm_driver/motor_manager.hpp"
#include "u1_arm_driver/trajectory_executor.hpp"

namespace u1_arm
{

// 状态发布节点所需配置 (由命令节点声明参数后注入, 避免 yaml 重复)
struct PublishConfig
{
  std::vector<std::string> joint_names;
  int udp_cycle_ms{5};
  int force_coordinate{0};
  bool joint_speed_enable{true};
};

// ============================ 命令节点 ============================
class U1ArmDriver : public rclcpp::Node
{
public:
  explicit U1ArmDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  std::shared_ptr<MotorManager> motors() const {return motors_;}
  std::shared_ptr<TrajectoryExecutor> executor() const {return exec_;}
  std::shared_ptr<Kinematics> kinematics() const {return kin_;}
  PublishConfig publish_config() const {return pub_cfg_;}
  int arm_dof() const {return static_cast<int>(motor_cfgs_.size());}

private:
  // ---- 初始化 ----
  std::vector<MotorConfig> load_motor_configs();
  void try_build_kinematics(const std::string & urdf_path);
  void setup_motion_topics();     // 运动/示教/急停等已实现
  void setup_query_topics();      // 状态/版本/坐标系/realtime_push 查询
  void setup_stub_topics();       // 未支持硬件功能打桩 (力/夹爪/手/升降/扩展/Modbus/轨迹/工程)
  void control_tick();            // 200Hz: 驱动 TrajectoryExecutor::tick
  void watchdog_tick();           // 500ms: 反馈超时检测

  // ---- 助手 ----
  // 建立 base+"_cmd"(MsgT 订阅) 与 base+"_result"(Bool 发布) 成对话题;
  // 回调执行 fn 并把返回值发到 result。fn: bool(const MsgT::SharedPtr &)
  template<typename MsgT, typename Fn>
  void add_bool_cmd(const std::string & base, Fn fn, rclcpp::CallbackGroup::SharedPtr group)
  {
    auto pub = create_publisher<std_msgs::msg::Bool>(
      "u1_arm/" + base + "_result", rclcpp::ParametersQoS());
    bool_results_[base] = pub;
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = group;
    auto sub = create_subscription<MsgT>(
      "u1_arm/" + base + "_cmd", rclcpp::ParametersQoS(),
      [this, base, fn](const typename MsgT::SharedPtr msg) {
        bool ok = false;
        try {
          ok = fn(msg);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "%s 回调异常: %s", base.c_str(), e.what());
        }
        publish_bool(base, ok);
      }, opt);
    subs_.push_back(sub);
  }
  void publish_bool(const std::string & base, bool ok);

  // 仅订阅 base+"_cmd"(MsgT), 不发布 _result (用于 rm 中无结果的透传/即发即忘命令)
  template<typename MsgT, typename Fn>
  void add_cmd_noresult(const std::string & base, Fn fn, rclcpp::CallbackGroup::SharedPtr group)
  {
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = group;
    subs_.push_back(
      create_subscription<MsgT>(
        "u1_arm/" + base + "_cmd", rclcpp::ParametersQoS(),
        [this, base, fn](const typename MsgT::SharedPtr msg) {
          try {
            fn(msg);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "%s 回调异常: %s", base.c_str(), e.what());
          }
        }, opt));
  }

  // 打桩查询: base+"_cmd"(CmdT) 触发 -> 发布默认 ResT(state=false) 到 base+"_result"
  template<typename CmdT, typename ResT>
  void add_stub_query(const std::string & base, rclcpp::CallbackGroup::SharedPtr group)
  {
    auto pub = create_publisher<ResT>("u1_arm/" + base + "_result", rclcpp::ParametersQoS());
    other_results_[base] = pub;
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = group;
    subs_.push_back(
      create_subscription<CmdT>(
        "u1_arm/" + base + "_cmd", rclcpp::ParametersQoS(),
        [this, base, pub](const typename CmdT::SharedPtr) {
          RCLCPP_WARN_ONCE(get_logger(), "%s: u1_arm 无此硬件, 返回默认(state=false)", base.c_str());
          pub->publish(ResT());
        }, opt));
  }
  // 打桩命令: base+"_cmd"(MsgT) 触发 -> WARN 并回 result=false
  template<typename MsgT>
  void add_stub_bool(const std::string & base, rclcpp::CallbackGroup::SharedPtr group)
  {
    add_bool_cmd<MsgT>(
      base, [this, base](const typename MsgT::SharedPtr) {
        RCLCPP_WARN_ONCE(get_logger(), "%s: u1_arm 无此硬件, 返回 false", base.c_str());
        return false;
      }, group);
  }

  // ---- 运动回调 ----
  bool on_movej(const std::vector<double> & joint, uint8_t speed, bool block);
  bool on_movej_canfd(const std::vector<double> & joint);   // 透传
  bool run_cartesian(std::vector<std::vector<double>> pts, bool block);
  // 笛卡尔 jog 示教: axis type 0/1/2 = x/y/z 或 rx/ry/rz, dir 0 负/1 正
  bool run_pos_teach(uint8_t axis, uint8_t dir, uint8_t speed, bool orientation);

  // ---- 共享臂对象 ----
  std::shared_ptr<CanBus> bus_;
  std::shared_ptr<MotorManager> motors_;
  std::shared_ptr<TrajectoryExecutor> exec_;
  std::shared_ptr<Kinematics> kin_;   // 可为空(URDF 缺失时笛卡尔运动不可用)
  std::vector<MotorConfig> motor_cfgs_;
  PublishConfig pub_cfg_;
  bool estop_disable_motors_{false};
  double tip_frame_registry_{0.0};  // 占位, 见下方软件坐标系
  std::string cur_work_frame_{"Base"};
  std::string cur_tool_frame_{"Arm_Tip"};

  // ---- 定时器/回调组 ----
  rclcpp::CallbackGroup::SharedPtr motion_group_;
  rclcpp::CallbackGroup::SharedPtr query_group_;
  rclcpp::CallbackGroup::SharedPtr stub_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // ---- 话题持有 ----
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> bool_results_;
  // 非 Bool 结果发布器 (查询类)
  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> other_results_;

  static constexpr double kMotionTimeoutS = 30.0;
};

// ============================ 状态发布节点 ============================
class U1StatePublisher : public rclcpp::Node
{
public:
  U1StatePublisher(
    std::shared_ptr<MotorManager> motors,
    std::shared_ptr<TrajectoryExecutor> exec,
    std::shared_ptr<Kinematics> kin,
    PublishConfig cfg,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_tick();

  std::shared_ptr<MotorManager> motors_;
  std::shared_ptr<TrajectoryExecutor> exec_;
  std::shared_ptr<Kinematics> kin_;
  PublishConfig cfg_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr arm_position_pub_;
  rclcpp::PublisherBase::SharedPtr joint_speed_pub_;
  rclcpp::PublisherBase::SharedPtr joint_temperature_pub_;
  rclcpp::PublisherBase::SharedPtr joint_current_pub_;
  rclcpp::PublisherBase::SharedPtr joint_voltage_pub_;
  rclcpp::PublisherBase::SharedPtr joint_en_flag_pub_;
  rclcpp::PublisherBase::SharedPtr joint_error_code_pub_;
  rclcpp::PublisherBase::SharedPtr joint_pose_euler_pub_;
};

}  // namespace u1_arm

#endif  // U1_ARM_DRIVER__U1_ARM_NODES_HPP_
