#include "u1_arm_driver/can_bus.hpp"

#include <array>
#include <chrono>

namespace u1_arm
{

using namespace std::chrono_literals;
using drivers::socketcan::CanId;
using drivers::socketcan::FrameType;
using drivers::socketcan::StandardFrame;

CanBus::CanBus(const std::string & interface, RxCallback on_frame)
: interface_(interface), on_frame_(std::move(on_frame))
{
  receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(interface_, false);
  sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(interface_, false);
  running_ = true;
  receive_thread_ = std::thread(&CanBus::receive_loop, this);
}

CanBus::~CanBus()
{
  running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
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
    return true;
  } catch (const std::exception &) {
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
        continue;
      }
      if (on_frame_) {
        on_frame_(can_id.identifier(), buf.data(), static_cast<uint8_t>(can_id.length()));
      }
    } catch (const drivers::socketcan::SocketCanTimeout &) {
      // 超时用于响应退出标志, 正常情况
    } catch (const std::exception &) {
      // 总线瞬时异常: 忽略, 由上层 watchdog 通过反馈超时发现持续故障
    }
  }
}

}  // namespace u1_arm
