// PiperX 机械臂 CAN 驱动节点
#ifndef AUTORUNNER_DRIVER__DRIVER_NODE_HPP_
#define AUTORUNNER_DRIVER__DRIVER_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

#include "can_msgs/msg/frame.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "autorunner_ros_interfaces/action/move_c.hpp"
#include "autorunner_ros_interfaces/action/move_l.hpp"
#include "autorunner_ros_interfaces/action/move_p.hpp"
#include "autorunner_ros_interfaces/msg/armpose.hpp"
#include "autorunner_ros_interfaces/msg/armstatus.hpp"
#include "autorunner_ros_interfaces/msg/collisionlevel.hpp"
#include "autorunner_ros_interfaces/msg/endvelacc.hpp"
#include "autorunner_ros_interfaces/msg/jointangle.hpp"
#include "autorunner_ros_interfaces/msg/jointcurrent.hpp"
#include "autorunner_ros_interfaces/msg/jointerrorcode.hpp"
#include "autorunner_ros_interfaces/msg/jointlimit.hpp"
#include "autorunner_ros_interfaces/msg/jointmaxacc.hpp"
#include "autorunner_ros_interfaces/msg/jointmit.hpp"
#include "autorunner_ros_interfaces/msg/jointmotorpos.hpp"
#include "autorunner_ros_interfaces/msg/jointspeed.hpp"
#include "autorunner_ros_interfaces/msg/jointtemperature.hpp"
#include "autorunner_ros_interfaces/msg/jointvoltage.hpp"
#include "autorunner_ros_interfaces/msg/setresponse.hpp"
#include "autorunner_ros_interfaces/msg/stop.hpp"
#include "autorunner_ros_interfaces/srv/clear_joint_error.hpp"
#include "autorunner_ros_interfaces/srv/enable_joint.hpp"
#include "autorunner_ros_interfaces/srv/query_collision_level.hpp"
#include "autorunner_ros_interfaces/srv/query_end_vel_acc.hpp"
#include "autorunner_ros_interfaces/srv/query_joint_limit.hpp"
#include "autorunner_ros_interfaces/srv/query_joint_max_acc.hpp"
#include "autorunner_ros_interfaces/srv/set_collision_level.hpp"
#include "autorunner_ros_interfaces/srv/set_end_vel_acc.hpp"
#include "autorunner_ros_interfaces/srv/set_joint_acc.hpp"
#include "autorunner_ros_interfaces/srv/set_joint_limit.hpp"
#include "autorunner_ros_interfaces/srv/set_joint_zero.hpp"
#include "autorunner_ros_interfaces/srv/set_motion_ctrl.hpp"

#include "autorunner_driver/pending_response.hpp"
#include "autorunner_driver/protocol/assembler.hpp"
#include "autorunner_driver/protocol/decoder.hpp"
#include "autorunner_driver/protocol/encoder.hpp"

namespace autorunner_driver
{

namespace msgs = autorunner_ros_interfaces::msg;
namespace srvs = autorunner_ros_interfaces::srv;
namespace acts = autorunner_ros_interfaces::action;
namespace protocol = autorunner::protocol;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

// ---- service 应答载荷 (收帧线程 -> service 线程) ----
struct SetRespPayload
{
  bool zero_set_success{false};
};
struct JointLimitPayload
{
  double max_angle{0}, min_angle{0}, max_speed{0};
};
struct JointMaxAccPayload
{
  double max_acc{0};
};
struct EndVelAccPayload
{
  double max_linear_vel{0}, max_angular_vel{0}, max_linear_acc{0}, max_angular_acc{0};
};
struct CollisionPayload
{
  std::array<uint8_t, 6> level{};
};

// ---- action 运动状态机阶段 ----
enum class MotionPhase
{
  kIdle,
  kWaitStart,   // 已下发目标, 等待 byte4 变为"未到达"确认开始 (或最小运动时间到)
  kWaitArrive,  // 等待 byte4 变为"到达"
};

class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DriverNode() override;

private:
  // ---- CAN 收发 ----
  void receive_loop();
  void dispatch(uint32_t id, const uint8_t * data, uint8_t dlc);
  bool send_frame(const protocol::RawFrame & frame);
  bool send_frames(const std::vector<protocol::RawFrame> & frames);

  // ---- 初始化序列: 0x471 使能全部 -> 0x151 进 CAN 模式 ----
  void initialize_arm();

  // ---- Topic 回调 (仅保留急停/MIT) ----
  void stop_callback(const msgs::Stop::SharedPtr msg);
  void joint_mit_callback(const msgs::Jointmit::SharedPtr msg);

  // ---- Service 回调 ----
  void enable_joint_service(
    const std::shared_ptr<srvs::EnableJoint::Request> req,
    std::shared_ptr<srvs::EnableJoint::Response> res);
  void set_joint_zero_service(
    const std::shared_ptr<srvs::SetJointZero::Request> req,
    std::shared_ptr<srvs::SetJointZero::Response> res);
  void clear_joint_error_service(
    const std::shared_ptr<srvs::ClearJointError::Request> req,
    std::shared_ptr<srvs::ClearJointError::Response> res);
  void set_joint_acc_service(
    const std::shared_ptr<srvs::SetJointAcc::Request> req,
    std::shared_ptr<srvs::SetJointAcc::Response> res);
  void query_joint_limit_service(
    const std::shared_ptr<srvs::QueryJointLimit::Request> req,
    std::shared_ptr<srvs::QueryJointLimit::Response> res);
  void query_joint_max_acc_service(
    const std::shared_ptr<srvs::QueryJointMaxAcc::Request> req,
    std::shared_ptr<srvs::QueryJointMaxAcc::Response> res);
  void query_end_vel_acc_service(
    const std::shared_ptr<srvs::QueryEndVelAcc::Request> req,
    std::shared_ptr<srvs::QueryEndVelAcc::Response> res);
  void query_collision_level_service(
    const std::shared_ptr<srvs::QueryCollisionLevel::Request> req,
    std::shared_ptr<srvs::QueryCollisionLevel::Response> res);
  void set_joint_limit_service(
    const std::shared_ptr<srvs::SetJointLimit::Request> req,
    std::shared_ptr<srvs::SetJointLimit::Response> res);
  void set_end_vel_acc_service(
    const std::shared_ptr<srvs::SetEndVelAcc::Request> req,
    std::shared_ptr<srvs::SetEndVelAcc::Response> res);
  void set_collision_level_service(
    const std::shared_ptr<srvs::SetCollisionLevel::Request> req,
    std::shared_ptr<srvs::SetCollisionLevel::Response> res);
  void set_motion_ctrl_service(
    const std::shared_ptr<srvs::SetMotionCtrl::Request> req,
    std::shared_ptr<srvs::SetMotionCtrl::Response> res);
  void emergency_stop_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ---- Action: MoveP / MoveL / MoveC / MoveJ(FollowJointTrajectory) ----
  template<typename ActionT>
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename ActionT::Goal> goal);
  template<typename ActionT>
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle);

  void execute_move_p(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveP>> handle);
  void execute_move_l(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveL>> handle);
  void execute_move_c(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<acts::MoveC>> handle);
  void execute_move_joint(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> handle);

  // 笛卡尔运动共用逻辑 (MoveP/MoveL): 下发目标 + 模式, 跑状态机
  template<typename ActionT>
  void run_cartesian_motion(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
    const protocol::PoseTargetCmd & target, uint8_t move_mode, uint8_t speed);

  // 运动状态机等待循环; 返回结束时 arm_status(0=正常到达), 或哨兵表示取消/超时
  // result_arm_status 出参填写结束时机械臂状态; 返回 true=到达 false=中止/取消
  bool wait_motion_done(
    std::function<bool()> is_canceling,
    std::function<void()> publish_feedback, uint8_t & result_arm_status,
    bool & canceled);

  // ---- 反馈发布 ----
  void publish_joint_states(const std::array<double, 6> & joint);
  void high_speed_timer_callback();
  void low_speed_timer_callback();
  void watchdog_callback();

  // ---- 协议 ----
  std::unique_ptr<protocol::Encoder> encoder_;
  std::unique_ptr<protocol::Decoder> decoder_;
  protocol::TripletAssembler end_pose_assembler_;
  protocol::TripletAssembler joint_angle_assembler_;

  std::mutex snapshot_mutex_;
  protocol::DriverHighSpeedSnapshot high_snapshot_;
  protocol::DriverLowSpeedSnapshot low_snapshot_;
  std::array<double, 6> last_end_pose_{};   // 最近一次末端位姿反馈 (movec 起点/feedback)
  std::array<double, 6> last_joint_angle_{};  // 最近一次关节角反馈 (feedback)

  // ---- SocketCAN ----
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::mutex send_mutex_;
  std::thread receive_thread_;
  std::atomic<bool> running_{false};

  // ---- 参数 ----
  std::vector<std::string> joint_names_;
  uint8_t install_pos_{1};
  uint8_t default_speed_{20};
  bool publish_raw_frames_{false};
  int init_retry_count_{3};
  std::chrono::milliseconds response_timeout_{1000};
  std::chrono::milliseconds motion_timeout_{30000};
  std::chrono::milliseconds min_move_time_{300};

  // ---- 看门狗 ----
  std::atomic<int64_t> last_status_time_ns_{0};

  // ---- service 应答等待器 (匹配 key 见注释) ----
  PendingResponse<SetRespPayload> pending_enable_;       // 0x476 byte0==0x71
  PendingResponse<SetRespPayload> pending_setjoint_;     // 0x476 byte0==0x75
  PendingResponse<JointLimitPayload> pending_jointlimit_;  // 0x473
  PendingResponse<JointMaxAccPayload> pending_jointacc_;   // 0x47C
  PendingResponse<EndVelAccPayload> pending_endvelacc_;    // 0x478
  PendingResponse<CollisionPayload> pending_collision_;    // 0x47B
  std::atomic<uint8_t> expected_joint_num_limit_{0};       // 0x473 关节号过滤
  std::atomic<uint8_t> expected_joint_num_acc_{0};         // 0x47C 关节号过滤

  // ---- action 运动跟踪 (收帧线程写, execute 线程读) ----
  std::atomic<bool> motion_active_{false};
  std::atomic<uint8_t> motion_status_{1};    // 0x2A1 byte4: 0到达 1未到达
  std::atomic<uint8_t> motion_arm_status_{0};  // 0x2A1 byte1
  std::mutex motion_mutex_;
  std::condition_variable motion_cv_;

  // ---- 订阅 (保留急停/MIT) ----
  rclcpp::Subscription<msgs::Stop>::SharedPtr stop_sub_;
  rclcpp::Subscription<msgs::Jointmit>::SharedPtr joint_mit_sub_;

  // ---- Service 句柄 ----
  rclcpp::Service<srvs::EnableJoint>::SharedPtr enable_joint_srv_;
  rclcpp::Service<srvs::SetJointZero>::SharedPtr set_joint_zero_srv_;
  rclcpp::Service<srvs::ClearJointError>::SharedPtr clear_joint_error_srv_;
  rclcpp::Service<srvs::SetJointAcc>::SharedPtr set_joint_acc_srv_;
  rclcpp::Service<srvs::QueryJointLimit>::SharedPtr query_joint_limit_srv_;
  rclcpp::Service<srvs::QueryJointMaxAcc>::SharedPtr query_joint_max_acc_srv_;
  rclcpp::Service<srvs::QueryEndVelAcc>::SharedPtr query_end_vel_acc_srv_;
  rclcpp::Service<srvs::QueryCollisionLevel>::SharedPtr query_collision_level_srv_;
  rclcpp::Service<srvs::SetJointLimit>::SharedPtr set_joint_limit_srv_;
  rclcpp::Service<srvs::SetEndVelAcc>::SharedPtr set_end_vel_acc_srv_;
  rclcpp::Service<srvs::SetCollisionLevel>::SharedPtr set_collision_level_srv_;
  rclcpp::Service<srvs::SetMotionCtrl>::SharedPtr set_motion_ctrl_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;

  // ---- Action 句柄 ----
  rclcpp_action::Server<acts::MoveP>::SharedPtr move_p_server_;
  rclcpp_action::Server<acts::MoveL>::SharedPtr move_l_server_;
  rclcpp_action::Server<acts::MoveC>::SharedPtr move_c_server_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr move_joint_server_;

  // ---- 发布 ----
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<msgs::Armstatus>::SharedPtr arm_status_pub_;
  rclcpp::Publisher<msgs::Armpose>::SharedPtr arm_position_pub_;
  rclcpp::Publisher<msgs::Jointangle>::SharedPtr joint_angle_pub_;
  rclcpp::Publisher<msgs::Jointspeed>::SharedPtr joint_speed_pub_;
  rclcpp::Publisher<msgs::Jointcurrent>::SharedPtr joint_current_pub_;
  rclcpp::Publisher<msgs::Jointmotorpos>::SharedPtr joint_motor_pos_pub_;
  rclcpp::Publisher<msgs::Jointvoltage>::SharedPtr joint_voltage_pub_;
  rclcpp::Publisher<msgs::Jointtemperature>::SharedPtr joint_temperature_pub_;
  rclcpp::Publisher<msgs::Jointerrorcode>::SharedPtr joint_error_code_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr raw_rx_pub_;

  // ---- 回调组与定时器 ----
  rclcpp::CallbackGroup::SharedPtr cmd_group_;      // topic (急停/MIT)
  rclcpp::CallbackGroup::SharedPtr service_group_;  // MutuallyExclusive: service 串行
  rclcpp::CallbackGroup::SharedPtr action_group_;   // Reentrant: action handle_*
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::TimerBase::SharedPtr high_speed_timer_;
  rclcpp::TimerBase::SharedPtr low_speed_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace autorunner_driver

#endif  // AUTORUNNER_DRIVER__DRIVER_NODE_HPP_
