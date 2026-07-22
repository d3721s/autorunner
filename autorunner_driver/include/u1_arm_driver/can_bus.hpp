// SocketCAN 封装: 独立收帧线程 + 互斥发送 —— 供 u1_arm 达妙电机驱动使用
// 模式参照 autorunner_driver 的 receive_loop/send, 但与 ROS 节点解耦,
// 反馈通过回调上抛(回调在 rx 线程中执行, 需自行保证线程安全)。
#ifndef U1_ARM_DRIVER__CAN_BUS_HPP_
#define U1_ARM_DRIVER__CAN_BUS_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

#include "u1_arm_driver/protocol/damiao_protocol.hpp"

namespace u1_arm
{

class CanBus
{
public:
  // 收到一帧标准数据帧时回调: (can_id, data, dlc)
  using RxCallback = std::function<void (uint32_t, const uint8_t *, uint8_t)>;

  // 打开接口失败抛 std::runtime_error(由 SocketCanReceiver/Sender 抛出)
  explicit CanBus(const std::string & interface, RxCallback on_frame);
  ~CanBus();

  CanBus(const CanBus &) = delete;
  CanBus & operator=(const CanBus &) = delete;

  // 线程安全; 发送失败(总线异常)返回 false
  bool send(const protocol::CanFrame & frame);

  const std::string & interface() const {return interface_;}

private:
  void receive_loop();

  std::string interface_;
  RxCallback on_frame_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::mutex send_mutex_;
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
};

}  // namespace u1_arm

#endif  // U1_ARM_DRIVER__CAN_BUS_HPP_
