// u1_arm 命令节点: 未支持硬件功能的打桩话题 (镜像 rm_driver 接口存在性)。
// 六维力 / 夹爪 / 灵巧手 / 升降 / 扩展关节 / Modbus / 轨迹文件 / 在线编程 /
// 工具电压 / 系统错误清除。cmd 回调一律 WARN 并发 result=false(或默认 state=false)。
// 对应 udp_* 状态话题创建但从不发布 (与 rm 关闭这些外设时行为一致)。
#include "u1_arm_driver/u1_arm_nodes.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include "autorunner_ros_interfaces/msg/gripperpick.hpp"
#include "autorunner_ros_interfaces/msg/gripperset.hpp"
#include "autorunner_ros_interfaces/msg/handangle.hpp"
#include "autorunner_ros_interfaces/msg/handforce.hpp"
#include "autorunner_ros_interfaces/msg/handposture.hpp"
#include "autorunner_ros_interfaces/msg/handseq.hpp"
#include "autorunner_ros_interfaces/msg/handspeed.hpp"
#include "autorunner_ros_interfaces/msg/handstatus.hpp"
#include "autorunner_ros_interfaces/msg/liftspeed.hpp"
#include "autorunner_ros_interfaces/msg/liftheight.hpp"
#include "autorunner_ros_interfaces/msg/liftstate.hpp"
#include "autorunner_ros_interfaces/msg/expandpos.hpp"
#include "autorunner_ros_interfaces/msg/expandstate.hpp"
#include "autorunner_ros_interfaces/msg/forcepositionmove.hpp"
#include "autorunner_ros_interfaces/msg/forcepositionmovejoint.hpp"
#include "autorunner_ros_interfaces/msg/forcepositionmovepose.hpp"
#include "autorunner_ros_interfaces/msg/setforceposition.hpp"
#include "autorunner_ros_interfaces/msg/sixforce.hpp"
#include "autorunner_ros_interfaces/msg/rmerr.hpp"
#include "autorunner_ros_interfaces/msg/armcurrentstatus.hpp"
#include "autorunner_ros_interfaces/msg/rmplusbase.hpp"
#include "autorunner_ros_interfaces/msg/rmplusstate.hpp"
#include "autorunner_ros_interfaces/msg/udpliftstate.hpp"
#include "autorunner_ros_interfaces/msg/udpexpandstate.hpp"
#include "autorunner_ros_interfaces/msg/alohastate.hpp"
// 四代控制器组
#include "autorunner_ros_interfaces/msg/gettrajectorylist.hpp"
#include "autorunner_ros_interfaces/msg/trajectorylist.hpp"
#include "autorunner_ros_interfaces/msg/rs485params.hpp"
#include "autorunner_ros_interfaces/msg/modbustcpmasterinfo.hpp"
#include "autorunner_ros_interfaces/msg/modbustcpmasterupdata.hpp"
#include "autorunner_ros_interfaces/msg/modbustcpmasterlist.hpp"
#include "autorunner_ros_interfaces/msg/mastername.hpp"
#include "autorunner_ros_interfaces/msg/getmodbustcpmasterlist.hpp"
#include "autorunner_ros_interfaces/msg/modbusrtureadparams.hpp"
#include "autorunner_ros_interfaces/msg/modbusrtuwriteparams.hpp"
#include "autorunner_ros_interfaces/msg/modbustcpreadparams.hpp"
#include "autorunner_ros_interfaces/msg/modbustcpwriteparams.hpp"
#include "autorunner_ros_interfaces/msg/modbusreaddata.hpp"
#include "autorunner_ros_interfaces/msg/programrunstate.hpp"
#include "autorunner_ros_interfaces/msg/flowchartrunstate.hpp"
#include "autorunner_ros_interfaces/msg/sendproject.hpp"

namespace u1_arm
{

namespace msgs = autorunner_ros_interfaces::msg;

void U1ArmDriver::setup_stub_topics()
{
  auto g = stub_group_;

  // ---------- 工具电压 / 系统错误 ----------
  add_stub_bool<std_msgs::msg::UInt16>("set_tool_voltage", g);
  add_bool_cmd<std_msgs::msg::Empty>(
    "clear_system_err",
    [](const std_msgs::msg::Empty::SharedPtr) {return true;}, g);

  // ---------- 力位混合 (无力传感器) ----------
  add_bool_cmd<std_msgs::msg::Empty>(
    "start_force_position_move",
    [this](const std_msgs::msg::Empty::SharedPtr) {
      RCLCPP_WARN_ONCE(get_logger(), "力位混合: u1_arm 无力传感器, 返回 false");
      return false;
    }, g);
  add_bool_cmd<std_msgs::msg::Empty>(
    "stop_force_position_move",
    [](const std_msgs::msg::Empty::SharedPtr) {return true;}, g);
  add_bool_cmd<std_msgs::msg::Empty>(
    "stop_force_postion",
    [](const std_msgs::msg::Empty::SharedPtr) {return true;}, g);
  add_stub_bool<msgs::Setforceposition>("set_force_postion", g);
  // 力位混合运动指令 (无 _result, 仅订阅) —— 保留订阅以匹配接口
  {
    rclcpp::SubscriptionOptions opt; opt.callback_group = g;
    subs_.push_back(
      create_subscription<msgs::Forcepositionmovejoint>(
        "u1_arm/force_position_move_joint_cmd", rclcpp::ParametersQoS(),
        [](const msgs::Forcepositionmovejoint::SharedPtr) {}, opt));
    subs_.push_back(
      create_subscription<msgs::Forcepositionmovepose>(
        "u1_arm/force_position_move_pose_cmd", rclcpp::ParametersQoS(),
        [](const msgs::Forcepositionmovepose::SharedPtr) {}, opt));
    subs_.push_back(
      create_subscription<msgs::Forcepositionmove>(
        "u1_arm/force_position_move_cmd", rclcpp::ParametersQoS(),
        [](const msgs::Forcepositionmove::SharedPtr) {}, opt));
  }

  // ---------- 六维力数据 (无传感器) ----------
  add_bool_cmd<std_msgs::msg::Empty>(
    "clear_force_data",
    [](const std_msgs::msg::Empty::SharedPtr) {return true;}, g);
  {
    auto f0 = create_publisher<msgs::Sixforce>(
      "u1_arm/get_force_data_result",
      rclcpp::ParametersQoS());
    auto f1 = create_publisher<msgs::Sixforce>(
      "u1_arm/get_zero_force_data_result",
      rclcpp::ParametersQoS());
    auto f2 = create_publisher<msgs::Sixforce>(
      "u1_arm/get_work_force_data_result",
      rclcpp::ParametersQoS());
    auto f3 = create_publisher<msgs::Sixforce>(
      "u1_arm/get_tool_force_data_result",
      rclcpp::ParametersQoS());
    other_results_["get_force_data"] = f0;
    other_results_["get_zero_force_data"] = f1;
    other_results_["get_work_force_data"] = f2;
    other_results_["get_tool_force_data"] = f3;
    rclcpp::SubscriptionOptions opt; opt.callback_group = g;
    subs_.push_back(
      create_subscription<std_msgs::msg::Empty>(
        "u1_arm/get_force_data_cmd", rclcpp::ParametersQoS(),
        [f0, f1, f2, f3](const std_msgs::msg::Empty::SharedPtr) {
          const msgs::Sixforce z;
          f0->publish(z); f1->publish(z); f2->publish(z); f3->publish(z);
        }, opt));
  }

  // ---------- 夹爪 ----------
  add_stub_bool<msgs::Gripperpick>("set_gripper_pick_on", g);
  add_stub_bool<msgs::Gripperpick>("set_gripper_pick", g);
  add_stub_bool<msgs::Gripperset>("set_gripper_position", g);

  // ---------- 灵巧手 ----------
  add_stub_bool<msgs::Handposture>("set_hand_posture", g);
  add_stub_bool<msgs::Handseq>("set_hand_seq", g);
  add_stub_bool<msgs::Handangle>("set_hand_angle", g);
  add_stub_bool<msgs::Handspeed>("set_hand_speed", g);
  add_stub_bool<msgs::Handforce>("set_hand_force", g);
  add_stub_bool<msgs::Handangle>("set_hand_follow_angle", g);
  add_stub_bool<msgs::Handangle>("set_hand_follow_pos", g);

  // ---------- 升降关节 ----------
  add_stub_bool<msgs::Liftspeed>("set_lift_speed", g);
  add_stub_bool<msgs::Liftheight>("set_lift_height", g);
  add_stub_query<std_msgs::msg::Empty, msgs::Liftstate>("get_lift_state", g);

  // ---------- 扩展关节 ----------
  add_stub_bool<std_msgs::msg::Int32>("set_expand_speed", g);
  add_stub_bool<msgs::Expandpos>("set_expand_pos", g);
  add_stub_query<std_msgs::msg::Empty, msgs::Expandstate>("get_expand_state", g);

  // ---------- 四代控制器: 轨迹文件 / 在线编程 ----------
  add_stub_query<msgs::Gettrajectorylist, msgs::Trajectorylist>("get_trajectory_file_list", g);
  add_stub_bool<std_msgs::msg::String>("set_run_trajectory", g);
  add_stub_bool<std_msgs::msg::String>("delete_trajectory_file", g);
  add_stub_bool<std_msgs::msg::String>("save_trajectory_file", g);
  add_stub_bool<msgs::Sendproject>("send_project", g);
  add_stub_query<std_msgs::msg::Empty, msgs::Programrunstate>("get_program_run_state", g);
  add_stub_query<std_msgs::msg::Empty, msgs::Flowchartrunstate>(
    "get_flowchart_program_run_state", g);

  // ---------- 四代控制器: Modbus / RS485 ----------
  add_stub_bool<msgs::RS485params>("set_controller_rs485_mode", g);
  add_stub_query<std_msgs::msg::Empty, msgs::RS485params>("get_controller_rs485_mode", g);
  add_stub_bool<msgs::RS485params>("set_tool_rs485_mode", g);
  add_stub_bool<msgs::Modbustcpmasterinfo>("add_modbus_tcp_master", g);
  add_stub_bool<msgs::Modbustcpmasterupdata>("update_modbus_tcp_master", g);
  add_stub_bool<msgs::Mastername>("delete_modbus_tcp_master", g);
  add_stub_query<msgs::Mastername, msgs::Modbustcpmasterinfo>("get_modbus_tcp_master", g);
  add_stub_query<msgs::Getmodbustcpmasterlist, msgs::Modbustcpmasterlist>(
    "get_modbus_tcp_master_list", g);
  add_stub_bool<std_msgs::msg::UInt16>("close_controller_rtu_modbus", g);
  add_stub_bool<msgs::Modbustcpmasterinfo>("set_controller_tcp_mode", g);
  add_stub_bool<std_msgs::msg::Empty>("close_controller_tcp_modbus", g);
  add_stub_query<msgs::Modbusrtureadparams, msgs::Modbusreaddata>("read_modbus_rtu_coils", g);
  add_stub_bool<msgs::Modbusrtuwriteparams>("write_modbus_rtu_coils", g);
  add_stub_query<msgs::Modbusrtureadparams, msgs::Modbusreaddata>(
    "read_modbus_rtu_input_status", g);
  add_stub_query<msgs::Modbusrtureadparams, msgs::Modbusreaddata>(
    "read_modbus_rtu_holding_registers", g);
  add_stub_bool<msgs::Modbusrtuwriteparams>("write_modbus_rtu_registers", g);
  add_stub_query<msgs::Modbusrtureadparams, msgs::Modbusreaddata>(
    "read_modbus_rtu_input_registers", g);
  add_stub_query<msgs::Modbustcpreadparams, msgs::Modbusreaddata>("read_modbus_tcp_coils", g);
  add_stub_bool<msgs::Modbustcpwriteparams>("write_modbus_tcp_coils", g);
  add_stub_query<msgs::Modbustcpreadparams, msgs::Modbusreaddata>(
    "read_modbus_tcp_input_status", g);
  add_stub_query<msgs::Modbustcpreadparams, msgs::Modbusreaddata>(
    "read_modbus_tcp_holding_registers", g);
  add_stub_bool<msgs::Modbustcpwriteparams>("write_modbus_tcp_registers", g);
  add_stub_query<msgs::Modbustcpreadparams, msgs::Modbusreaddata>(
    "read_modbus_tcp_input_registers", g);

  // get_tool_rs485_mode: cmd 无后缀, result 为 *_v4_result (rm 特例)
  {
    auto pub = create_publisher<msgs::RS485params>(
      "u1_arm/get_tool_rs485_mode_v4_result", rclcpp::ParametersQoS());
    other_results_["get_tool_rs485_mode_v4"] = pub;
    rclcpp::SubscriptionOptions opt; opt.callback_group = g;
    subs_.push_back(
      create_subscription<std_msgs::msg::Empty>(
        "u1_arm/get_tool_rs485_mode_cmd", rclcpp::ParametersQoS(),
        [pub](const std_msgs::msg::Empty::SharedPtr) {pub->publish(msgs::RS485params());}, opt));
  }

  // ---------- 从不发布的 udp_* 状态话题 (硬件缺省关闭) ----------
  auto qos = rclcpp::QoS(10);
  other_results_["udp_six_force"] =
    create_publisher<msgs::Sixforce>("u1_arm/udp_six_force", qos);
  other_results_["udp_six_zero_force"] =
    create_publisher<msgs::Sixforce>("u1_arm/udp_six_zero_force", qos);
  other_results_["udp_one_force"] =
    create_publisher<msgs::Sixforce>("u1_arm/udp_one_force", qos);
  other_results_["udp_one_zero_force"] =
    create_publisher<msgs::Sixforce>("u1_arm/udp_one_zero_force", qos);
  other_results_["udp_hand_status"] =
    create_publisher<msgs::Handstatus>("u1_arm/udp_hand_status", qos);
  other_results_["udp_arm_current_status"] =
    create_publisher<msgs::Armcurrentstatus>("u1_arm/udp_arm_current_status", qos);
  other_results_["udp_arm_coordinate"] =
    create_publisher<std_msgs::msg::UInt16>("u1_arm/udp_arm_coordinate", qos);
  other_results_["udp_rm_plus_base"] =
    create_publisher<msgs::Rmplusbase>("u1_arm/udp_rm_plus_base", qos);
  other_results_["udp_rm_plus_state"] =
    create_publisher<msgs::Rmplusstate>("u1_arm/udp_rm_plus_state", qos);
  other_results_["udp_lift_state"] =
    create_publisher<msgs::Udpliftstate>("u1_arm/udp_lift_state", qos);
  other_results_["udp_expand_state"] =
    create_publisher<msgs::Udpexpandstate>("u1_arm/udp_expand_state", qos);
  other_results_["udp_aloha_state"] =
    create_publisher<msgs::Alohastate>("u1_arm/udp_aloha_state", qos);
  // udp_rm_err: watchdog 反馈超时时发布
  other_results_["udp_rm_err"] = create_publisher<msgs::Rmerr>("u1_arm/udp_rm_err", qos);
}

}  // namespace u1_arm
