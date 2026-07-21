#include "rclcpp/rclcpp.hpp"

#include "autorunner_driver/driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 线程数 >=4: 收帧线程独立, 执行器需并行处理 timer/service(阻塞等待)/action
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  auto node = std::make_shared<autorunner_driver::DriverNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
