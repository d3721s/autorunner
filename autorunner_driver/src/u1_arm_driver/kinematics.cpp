#include "u1_arm_driver/kinematics.hpp"

#include <cmath>
#include <stdexcept>

#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace u1_arm
{

namespace
{

KDL::Frame pose_to_frame(const Pose & p)
{
  return KDL::Frame(
    KDL::Rotation::Quaternion(p.qx, p.qy, p.qz, p.qw),
    KDL::Vector(p.x, p.y, p.z));
}

Pose frame_to_pose(const KDL::Frame & f)
{
  Pose p;
  p.x = f.p.x();
  p.y = f.p.y();
  p.z = f.p.z();
  f.M.GetQuaternion(p.qx, p.qy, p.qz, p.qw);
  return p;
}

}  // namespace

Kinematics::Kinematics(
  const std::string & urdf_path,
  const std::string & base_link, const std::string & tip_link,
  const std::vector<double> & joint_lower, const std::vector<double> & joint_upper)
: lower_(joint_lower), upper_(joint_upper)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_path, tree)) {
    throw std::runtime_error("URDF 解析失败: " + urdf_path);
  }
  if (!tree.getChain(base_link, tip_link, chain_)) {
    throw std::runtime_error("提取链失败: " + base_link + " -> " + tip_link);
  }
  const auto n = chain_.getNrOfJoints();
  if (n != joint_lower.size() || n != joint_upper.size()) {
    throw std::runtime_error(
            "链关节数 " + std::to_string(n) + " 与限位表 " +
            std::to_string(joint_lower.size()) + " 不一致");
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  // 任务空间权重: 位置 xyz=1, 姿态 rxyz=0.1 -> 位置优先, 姿态尽力
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1.0, 1.0, 1.0, 0.1, 0.1, 0.1;
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, weights, 1e-5, 500);
}

size_t Kinematics::num_joints() const
{
  return chain_.getNrOfJoints();
}

Pose Kinematics::fk(const std::vector<double> & joints) const
{
  KDL::JntArray q(joints.size());
  for (size_t i = 0; i < joints.size(); ++i) {
    q(i) = joints[i];
  }
  KDL::Frame out;
  fk_solver_->JntToCart(q, out);
  return frame_to_pose(out);
}

void Kinematics::quat_to_euler(const Pose & p, double & roll, double & pitch, double & yaw)
{
  KDL::Rotation::Quaternion(p.qx, p.qy, p.qz, p.qw).GetRPY(roll, pitch, yaw);
}

void Kinematics::euler_to_quat(double roll, double pitch, double yaw, Pose & out)
{
  KDL::Rotation::RPY(roll, pitch, yaw).GetQuaternion(out.qx, out.qy, out.qz, out.qw);
}

std::optional<std::vector<double>> Kinematics::ik(
  const Pose & target, const std::vector<double> & seed) const
{
  const auto n = chain_.getNrOfJoints();
  KDL::JntArray q_init(n), q_out(n);
  for (size_t i = 0; i < n && i < seed.size(); ++i) {
    q_init(i) = seed[i];
  }

  if (ik_solver_->CartToJnt(q_init, pose_to_frame(target), q_out) < 0) {
    return std::nullopt;
  }
  std::vector<double> out(n);
  for (size_t i = 0; i < n; ++i) {
    // 归一到 [-pi, pi] 邻域后做限位校验
    double v = q_out(i);
    while (v > lower_[i] + 2 * M_PI) {v -= 2 * M_PI;}
    while (v < upper_[i] - 2 * M_PI) {v += 2 * M_PI;}
    if (v < lower_[i] - 1e-4 || v > upper_[i] + 1e-4) {
      return std::nullopt;
    }
    out[i] = v;
  }
  return out;
}

std::optional<std::vector<std::vector<double>>> Kinematics::cartesian_line(
  const std::vector<double> & start_joints, const Pose & target, double step_m) const
{
  const Pose start = fk(start_joints);
  const KDL::Frame f0 = pose_to_frame(start);
  const KDL::Frame f1 = pose_to_frame(target);

  const double dist = (f1.p - f0.p).Norm();
  // 姿态角变化也计入步数(等效弧长, 让纯旋转也能插值)
  KDL::Rotation r_rel = f0.M.Inverse() * f1.M;
  KDL::Vector axis;
  const double angle = r_rel.GetRotAngle(axis);
  const size_t steps = std::max<size_t>(
    2, static_cast<size_t>(std::ceil(std::max(dist, angle * 0.1) / step_m)));

  std::vector<std::vector<double>> out;
  out.reserve(steps);
  std::vector<double> seed = start_joints;
  for (size_t k = 1; k <= steps; ++k) {
    const double s = static_cast<double>(k) / steps;
    KDL::Frame f;
    f.p = f0.p + (f1.p - f0.p) * s;
    f.M = f0.M * KDL::Rotation::Rot2(axis, angle * s);
    auto q = ik(frame_to_pose(f), seed);
    if (!q) {return std::nullopt;}
    seed = *q;
    out.push_back(std::move(*q));
  }
  return out;
}

std::optional<std::vector<std::vector<double>>> Kinematics::cartesian_arc(
  const std::vector<double> & start_joints, const Pose & mid, const Pose & end,
  double step_m, int loop) const
{
  const Pose start = fk(start_joints);
  const KDL::Vector p1(start.x, start.y, start.z);
  const KDL::Vector p2(mid.x, mid.y, mid.z);
  const KDL::Vector p3(end.x, end.y, end.z);

  // 三点求圆心: 圆在 (p1,p2,p3) 平面内
  const KDL::Vector v21 = p2 - p1, v31 = p3 - p1;
  KDL::Vector n = v21 * v31;   // 平面法向
  const double n_norm = n.Norm();
  if (n_norm < 1e-9) {return std::nullopt;}  // 三点共线
  n = n / n_norm;

  // 圆心 = p1 + a*v21 + b*v31 (解自 |c-p1|=|c-p2|=|c-p3|)
  const double d21 = KDL::dot(v21, v21), d31 = KDL::dot(v31, v31);
  const double d2131 = KDL::dot(v21, v31);
  const double denom = 2.0 * (d21 * d31 - d2131 * d2131);
  if (std::abs(denom) < 1e-12) {return std::nullopt;}
  const double a = (d31 * (d21 - d2131)) / denom;
  const double b = (d21 * (d31 - d2131)) / denom;
  const KDL::Vector center = p1 + v21 * a + v31 * b;
  const double radius = (p1 - center).Norm();
  if (radius < 1e-6) {return std::nullopt;}

  // 平面内极角参数化
  const KDL::Vector u = (p1 - center) / radius;      // 0 角基准
  const KDL::Vector w = n * u;                       // 90° 方向
  auto angle_of = [&](const KDL::Vector & p) {
      const KDL::Vector r = p - center;
      return std::atan2(KDL::dot(r, w), KDL::dot(r, u));
    };
  double a2 = angle_of(p2), a3 = angle_of(p3);
  // 保证走向经过 p2: p2 角度必须在 0..a3 同向路径上
  if (a2 < 0) {a2 += 2 * M_PI;}
  if (a3 < 0) {a3 += 2 * M_PI;}
  double total = (a2 <= a3) ? a3 : a3 + 2 * M_PI;    // 逆向时补一圈
  total += loop * 2 * M_PI;

  const KDL::Frame f0 = pose_to_frame(start);
  const KDL::Frame f1 = pose_to_frame(end);
  KDL::Rotation r_rel = f0.M.Inverse() * f1.M;
  KDL::Vector rot_axis;
  const double rot_angle = r_rel.GetRotAngle(rot_axis);

  const double arc_len = radius * total;
  const size_t steps = std::max<size_t>(2, static_cast<size_t>(std::ceil(arc_len / step_m)));

  std::vector<std::vector<double>> out;
  out.reserve(steps);
  std::vector<double> seed = start_joints;
  for (size_t k = 1; k <= steps; ++k) {
    const double s = static_cast<double>(k) / steps;
    const double th = total * s;
    KDL::Frame f;
    f.p = center + (u * std::cos(th) + w * std::sin(th)) * radius;
    f.M = f0.M * KDL::Rotation::Rot2(rot_axis, rot_angle * s);
    auto q = ik(frame_to_pose(f), seed);
    if (!q) {return std::nullopt;}
    seed = *q;
    out.push_back(std::move(*q));
  }
  return out;
}

}  // namespace u1_arm
