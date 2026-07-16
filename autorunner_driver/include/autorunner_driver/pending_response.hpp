// 通用 CAN 应答等待器: service 回调下发指令后阻塞等待收帧线程投递的对应应答
#ifndef AUTORUNNER_DRIVER__PENDING_RESPONSE_HPP_
#define AUTORUNNER_DRIVER__PENDING_RESPONSE_HPP_

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>

namespace autorunner_driver
{

// 每个实例自带独立的 mutex + condition_variable, 内部锁绝不与节点其它锁嵌套:
//   - service 线程: arm() -> (发送) -> wait()   仅持自身锁阻塞
//   - 收帧线程 dispatch: notify()               仅持自身锁 O(1)
// 从而与 snapshot_mutex_ / send_mutex_ 无交叉, 不会死锁。
template<typename T>
class PendingResponse
{
public:
  // 等待前调用: 清空上次结果并进入待命状态
  void arm()
  {
    std::lock_guard<std::mutex> lock(m_);
    value_.reset();
    armed_ = true;
  }

  // 阻塞至命中或超时; 返回后自动解除待命
  std::optional<T> wait(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait_for(lock, timeout, [this] {return value_.has_value();});
    armed_ = false;
    return value_;
  }

  // 收帧线程投递应答; 未处于待命状态则丢弃 (避免残留污染下次等待)
  void notify(const T & v)
  {
    {
      std::lock_guard<std::mutex> lock(m_);
      if (!armed_) {return;}
      value_ = v;
    }
    cv_.notify_one();
  }

private:
  std::mutex m_;
  std::condition_variable cv_;
  std::optional<T> value_;
  bool armed_{false};
};

}  // namespace autorunner_driver

#endif  // AUTORUNNER_DRIVER__PENDING_RESPONSE_HPP_
