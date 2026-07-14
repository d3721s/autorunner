#include "autorunner_driver/protocol/assembler.hpp"

namespace autorunner::protocol
{

std::optional<std::array<double, 6>> TripletAssembler::feed(
  uint8_t part, double a, double b, Clock::time_point now)
{
  if (part > 2) {
    return std::nullopt;
  }

  // 半包超时: 丢弃陈旧的未收齐数据
  if (received_mask_ != 0 && now - first_time_ > timeout_) {
    reset();
  }
  // part0 视为新周期开始
  if (part == 0) {
    reset();
  }
  if (received_mask_ == 0) {
    first_time_ = now;
  }

  values_[part * 2] = a;
  values_[part * 2 + 1] = b;
  received_mask_ |= static_cast<uint8_t>(1u << part);

  if (received_mask_ == 0b111) {
    auto out = values_;
    reset();
    return out;
  }
  return std::nullopt;
}

void TripletAssembler::reset()
{
  received_mask_ = 0;
}

}  // namespace autorunner::protocol
