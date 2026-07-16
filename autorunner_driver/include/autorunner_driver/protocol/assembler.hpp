// 多帧组装: 三帧一组的末端位姿/关节角, 以及高低速反馈快照聚合
#ifndef AUTORUNNER_DRIVER__PROTOCOL__ASSEMBLER_HPP_
#define AUTORUNNER_DRIVER__PROTOCOL__ASSEMBLER_HPP_

#include <array>
#include <chrono>
#include <optional>

#include "autorunner_driver/protocol/decoder.hpp"
#include "autorunner_driver/protocol/frames.hpp"

namespace autorunner::protocol
{

// 通用三帧组装器: 收齐 {0,1,2} 即输出, 不依赖到达顺序;
// 收到 part0 视为新周期开始并重置; 超过 timeout 未收齐则丢弃半包。
class TripletAssembler
{
public:
  using Clock = std::chrono::steady_clock;

  explicit TripletAssembler(std::chrono::milliseconds timeout = std::chrono::milliseconds(50))
  : timeout_(timeout) {}

  // 输入一帧 (part 0~2, 每帧携带两个 double), 收齐返回 6 个值 [a0,b0,a1,b1,a2,b2]
  std::optional<std::array<double, 6>> feed(
    uint8_t part, double a, double b,
    Clock::time_point now = Clock::now());

private:
  void reset();

  std::chrono::milliseconds timeout_;
  std::array<double, 6> values_{};
  uint8_t received_mask_{0};
  Clock::time_point first_time_{};
};

// 高速反馈快照 (0x251~6): 到帧即更新, 由节点定时器拍快照发布
struct DriverHighSpeedSnapshot
{
  std::array<double, 6> speed{};     // rad/s
  std::array<double, 6> current{};   // A
  std::array<double, 6> position{};  // rad

  void update(const DriverHighSpeedFb & fb)
  {
    if (fb.joint_index >= 6) {return;}
    speed[fb.joint_index] = fb.speed;
    current[fb.joint_index] = fb.current;
    position[fb.joint_index] = fb.position;
  }
};

// 低速反馈快照 (0x261~6)
struct DriverLowSpeedSnapshot
{
  std::array<double, 6> voltage{};      // V
  std::array<double, 6> driver_temp{};  // ℃
  std::array<double, 6> motor_temp{};   // ℃
  std::array<double, 6> bus_current{};  // A
  std::array<uint8_t, 6> status{};

  void update(const DriverLowSpeedFb & fb)
  {
    if (fb.joint_index >= 6) {return;}
    voltage[fb.joint_index] = fb.voltage;
    driver_temp[fb.joint_index] = fb.driver_temp;
    motor_temp[fb.joint_index] = fb.motor_temp;
    bus_current[fb.joint_index] = fb.bus_current;
    status[fb.joint_index] = fb.status;
  }
};

}  // namespace autorunner::protocol

#endif  // AUTORUNNER_DRIVER__PROTOCOL__ASSEMBLER_HPP_
