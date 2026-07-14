// PiperX 机械臂 CAN 驱动节点
#ifndef AUTORUNNER_DRIVER__DRIVER_NODE_HPP_
#define AUTORUNNER_DRIVER__DRIVER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "autorunner_ros_interfaces/msg/armpose.hpp"
#include "autorunner_ros_interfaces/msg/armstatus.hpp"
#include "autorunner_ros_interfaces/msg/collisionlevel.hpp"
#include "autorunner_ros_interfaces/msg/endvelacc.hpp"
#include "autorunner_ros_interfaces/msg/jointangle.hpp"
#include "autorunner_ros_interfaces/msg/jointconfig.hpp"
#include "autorunner_ros_interfaces/msg/jointcurrent.hpp"
#include "autorunner_ros_interfaces/msg/jointenable.hpp"
#include "autorunner_ros_interfaces/msg/jointerrorcode.hpp"
#include "autorunner_ros_interfaces/msg/jointlimit.hpp"
#include "autorunner_ros_interfaces/msg/jointlimitquery.hpp"
#include "autorunner_ros_interfaces/msg/jointlimitset.hpp"
#include "autorunner_ros_interfaces/msg/jointmaxacc.hpp"
#include "autorunner_ros_interfaces/msg/jointmit.hpp"
#include "autorunner_ros_interfaces/msg/jointmotorpos.hpp"
#include "autorunner_ros_interfaces/msg/jointspeed.hpp"
#include "autorunner_ros_interfaces/msg/jointtemperature.hpp"
#include "autorunner_ros_interfaces/msg/jointvoltage.hpp"
#include "autorunner_ros_interfaces/msg/modectrl.hpp"
#include "autorunner_ros_interfaces/msg/motionctrl.hpp"
#include "autorunner_ros_interfaces/msg/movec.hpp"
#include "autorunner_ros_interfaces/msg/movej.hpp"
#include "autorunner_ros_interfaces/msg/movep.hpp"
#include "autorunner_ros_interfaces/msg/setresponse.hpp"
#include "autorunner_ros_interfaces/msg/stop.hpp"

#include "autorunner_driver/protocol/assembler.hpp"
#include "autorunner_driver/protocol/decoder.hpp"
#include "autorunner_driver/protocol/encoder.hpp"

namespace autorunner_driver
{

namespace msgs = autorunner_ros_interfaces::msg;
namespace protocol = autorunner::protocol;

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

  // ---- 命令回调 ----
  void movej_callback(const msgs::Movej::SharedPtr msg);
  void movep_callback(const msgs::Movep::SharedPtr msg);
  void movel_callback(const msgs::Movep::SharedPtr msg);
  void movec_callback(const msgs::Movec::SharedPtr msg);
  void stop_callback(const msgs::Stop::SharedPtr msg);
  void motion_ctrl_callback(const msgs::Motionctrl::SharedPtr msg);
  void mode_ctrl_callback(const msgs::Modectrl::SharedPtr msg);
  void enable_callback(const msgs::Jointenable::SharedPtr msg);
  void joint_config_callback(const msgs::Jointconfig::SharedPtr msg);
  void joint_limit_query_callback(const msgs::Jointlimitquery::SharedPtr msg);
  void joint_limit_set_callback(const msgs::Jointlimitset::SharedPtr msg);
  void collision_level_callback(const msgs::Collisionlevel::SharedPtr msg);
  void end_velacc_set_callback(const msgs::Endvelacc::SharedPtr msg);
  void joint_mit_callback(const msgs::Jointmit::SharedPtr msg);

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
  std::array<double, 6> last_end_pose_{};  // 最近一次末端位姿反馈 (movec 起点用)

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

  // ---- 看门狗 ----
  std::atomic<int64_t> last_status_time_ns_{0};

  // ---- 订阅 ----
  rclcpp::Subscription<msgs::Movej>::SharedPtr movej_sub_;
  rclcpp::Subscription<msgs::Movep>::SharedPtr movep_sub_;
  rclcpp::Subscription<msgs::Movep>::SharedPtr movel_sub_;
  rclcpp::Subscription<msgs::Movec>::SharedPtr movec_sub_;
  rclcpp::Subscription<msgs::Stop>::SharedPtr stop_sub_;
  rclcpp::Subscription<msgs::Motionctrl>::SharedPtr motion_ctrl_sub_;
  rclcpp::Subscription<msgs::Modectrl>::SharedPtr mode_ctrl_sub_;
  rclcpp::Subscription<msgs::Jointenable>::SharedPtr enable_sub_;
  rclcpp::Subscription<msgs::Jointconfig>::SharedPtr joint_config_sub_;
  rclcpp::Subscription<msgs::Jointlimitquery>::SharedPtr joint_limit_query_sub_;
  rclcpp::Subscription<msgs::Jointlimitset>::SharedPtr joint_limit_set_sub_;
  rclcpp::Subscription<msgs::Collisionlevel>::SharedPtr collision_level_sub_;
  rclcpp::Subscription<msgs::Endvelacc>::SharedPtr end_velacc_set_sub_;
  rclcpp::Subscription<msgs::Jointmit>::SharedPtr joint_mit_sub_;

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
  rclcpp::Publisher<msgs::Jointlimit>::SharedPtr joint_limit_pub_;
  rclcpp::Publisher<msgs::Jointmaxacc>::SharedPtr joint_max_acc_pub_;
  rclcpp::Publisher<msgs::Collisionlevel>::SharedPtr collision_level_state_pub_;
  rclcpp::Publisher<msgs::Endvelacc>::SharedPtr end_velacc_state_pub_;
  rclcpp::Publisher<msgs::Setresponse>::SharedPtr set_response_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movej_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movep_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movel_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movec_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_result_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr raw_rx_pub_;

  // ---- 定时器与回调组 ----
  rclcpp::CallbackGroup::SharedPtr cmd_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::TimerBase::SharedPtr high_speed_timer_;
  rclcpp::TimerBase::SharedPtr low_speed_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace autorunner_driver

#endif  // AUTORUNNER_DRIVER__DRIVER_NODE_HPP_
