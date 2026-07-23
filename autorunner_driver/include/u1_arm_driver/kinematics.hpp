// KDL 运动学: 从 URDF 提取 l0→l5 链 (5 关节 j1..j5)
// - FK: 关节角 -> 末端位姿 (quaternion / 欧拉角)
// - IK: LMA 加权(位置优先, 姿态尽力 —— 5 自由度无法全姿态跟踪)
// - 笛卡尔插值: MoveL 直线 / MoveC 三点圆弧, 生成逐 tick 关节采样序列
#ifndef U1_ARM_DRIVER__KINEMATICS_HPP_
#define U1_ARM_DRIVER__KINEMATICS_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "kdl/chaindynparam.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"

namespace u1_arm
{

struct Pose
{
  double x{0.0}, y{0.0}, z{0.0};        // m
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

class Kinematics
{
public:
  // 从 URDF 文件构建 base_link->tip_link 链; 失败抛 std::runtime_error
  Kinematics(
    const std::string & urdf_path,
    const std::string & base_link, const std::string & tip_link,
    const std::vector<double> & joint_lower, const std::vector<double> & joint_upper);

  size_t num_joints() const;

  Pose fk(const std::vector<double> & joints) const;
  // 位姿 -> 欧拉角 (ZYX: rx ry rz 按 rm 惯例返回 roll pitch yaw)
  static void quat_to_euler(const Pose & p, double & roll, double & pitch, double & yaw);
  // 欧拉角 -> 位姿四元数 (写入 out 的 q 分量, 不改位置)
  static void euler_to_quat(double roll, double pitch, double yaw, Pose & out);

  // 单点 IK; seed 为迭代初值(通常用当前关节角)。失败返回 nullopt。
  std::optional<std::vector<double>> ik(
    const Pose & target,
    const std::vector<double> & seed) const;

  // 关节侧重力补偿力矩(N·m), 使用 URDF inertial 参数和 base_link 坐标系下的重力向量。
  // 返回空向量表示动力学求解失败或输入关节数不匹配。
  std::vector<double> gravity_torques(const std::vector<double> & joints) const;

  // 直线插值: 从 start_joints 的 FK 位姿直线走到 target, 每步 step_m/step_rad,
  // 逐点 IK(种子链式传递)。任一点 IK 失败返回 nullopt。
  std::optional<std::vector<std::vector<double>>> cartesian_line(
    const std::vector<double> & start_joints, const Pose & target,
    double step_m) const;

  // 三点圆弧: 起点(由 start_joints FK 得出)->中间点->终点, loop 圈数(0=单段)
  std::optional<std::vector<std::vector<double>>> cartesian_arc(
    const std::vector<double> & start_joints, const Pose & mid, const Pose & end,
    double step_m, int loop) const;

private:
  KDL::Chain chain_;
  std::vector<double> lower_, upper_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  // LMA 权重: 位置 1.0, 姿态 0.1 —— 位置优先
  mutable std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  mutable std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
  mutable std::mutex solver_mutex_;
};

}  // namespace u1_arm

#endif  // U1_ARM_DRIVER__KINEMATICS_HPP_
