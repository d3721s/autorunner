#include "u1_arm_driver/can_bus.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace u1_arm
{

using namespace std::chrono_literals;
using drivers::socketcan::CanId;
using drivers::socketcan::FrameType;
using drivers::socketcan::StandardFrame;

namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("u1_arm.can_bus");
}

rclcpp::Clock & log_clock()
{
  static rclcpp::Clock clock(RCL_SYSTEM_TIME);
  return clock;
}

std::string format_frame(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  std::ostringstream out;
  out << "id=0x" << std::uppercase << std::hex << id << std::dec
      << " dlc=" << static_cast<int>(dlc) << " data=[";
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

std::string format_frame(const protocol::CanFrame & frame)
{
  return format_frame(frame.id, frame.data.data(), frame.dlc);
}

}  // namespace

CanBus::CanBus(const std::string & interface, RxCallback on_frame)
: interface_(interface), on_frame_(std::move(on_frame))
{
  receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(interface_, false);
  sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(interface_, false);
  running_ = true;
  receive_thread_ = std::thread(&CanBus::receive_loop, this);
  RCLCPP_INFO(logger(), "CAN 总线已打开: interface=%s, 接收线程已启动", interface_.c_str());
}

CanBus::~CanBus()
{
  RCLCPP_INFO(logger(), "CAN 总线关闭: interface=%s, 等待接收线程退出", interface_.c_str());
  running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  RCLCPP_INFO(logger(), "CAN 接收线程已退出: interface=%s", interface_.c_str());
}

bool CanBus::send(const protocol::CanFrame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  try {
    // 用 raw 构造 (不做截断检查): 达妙广播 ID 0x7FF 是合法标准帧, 但
    // ros2_socketcan 的带类型标签构造函数会拒绝 SFF 范围顶部的 ID。
    // frame.id 的 bit29~31 恒为 0, 故等价于标准数据帧。
    const CanId id(frame.id, 0);
    sender_->send(frame.data.data(), frame.dlc, id, 10ms);
    RCLCPP_DEBUG(logger(), "CAN 下发成功: %s", format_frame(frame).c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger(), "CAN 下发失败: %s error=%s", format_frame(frame).c_str(), e.what());
    return false;
  }
}

void CanBus::receive_loop()
{
  std::array<uint8_t, 8> buf{};
  while (running_) {
    try {
      buf.fill(0);
      const auto can_id = receiver_->receive(buf.data(), 100ms);
      if (can_id.frame_type() != FrameType::DATA || can_id.is_extended()) {
        RCLCPP_DEBUG_THROTTLE(
          logger(), log_clock(), 5000,
          "忽略非标准数据 CAN 帧: id=0x%X extended=%s",
          can_id.identifier(), can_id.is_extended() ? "true" : "false");
        continue;
      }
      RCLCPP_DEBUG_THROTTLE(
        logger(), log_clock(), 1000, "CAN 收到: %s",
        format_frame(
          can_id.identifier(), buf.data(),
          static_cast<uint8_t>(can_id.length())).c_str());
      if (on_frame_) {
        on_frame_(can_id.identifier(), buf.data(), static_cast<uint8_t>(can_id.length()));
      }
    } catch (const drivers::socketcan::SocketCanTimeout &) {
      // 超时用于响应退出标志, 正常情况
    } catch (const std::exception & e) {
      // 总线瞬时异常: 忽略, 由上层 watchdog 通过反馈超时发现持续故障
      RCLCPP_WARN_THROTTLE(
        logger(), log_clock(), 5000, "CAN 接收异常: %s", e.what());
    }
  }
}

}  // namespace u1_arm
