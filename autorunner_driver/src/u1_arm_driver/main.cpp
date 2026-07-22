// u1_arm 驱动入口: 命令节点 + 状态发布节点, 共用一个 MultiThreadedExecutor(8线程)
// (与 rm_driver 的双节点单进程结构一致)。
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "u1_arm_driver/u1_arm_nodes.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto driver = std::make_shared<u1_arm::U1ArmDriver>();
  auto publisher = std::make_shared<u1_arm::U1StatePublisher>(
    driver->motors(), driver->executor(), driver->kinematics(), driver->publish_config());

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
  executor.add_node(driver);
  executor.add_node(publisher);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
