// DriverNode 的 service 回调实现: 带 CAN 应答等待的请求-响应
#include "autorunner_driver/driver_node.hpp"

#include <cmath>
#include <limits>

namespace autorunner_driver
{

namespace proto = autorunner::protocol;

// ---- 带应答: 电机使能 (0x471 -> 0x476 byte0=0x71) ----
void DriverNode::enable_joint_service(
  const std::shared_ptr<srvs::EnableJoint::Request> req,
  std::shared_ptr<srvs::EnableJoint::Response> res)
{
  proto::MotorEnableCmd cmd;
  cmd.joint_num = req->joint_num;
  cmd.enable = req->enable;

  pending_enable_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_enable_.wait(response_timeout_);
  if (resp) {
    res->success = true;
    res->message = req->enable ? "已使能" : "已失能";
  } else {
    res->success = false;
    res->message = "等待 0x476 应答超时";
  }
}

// ---- 带应答: 关节零点 (0x475 set_zero -> 0x476 byte0=0x75, byte1) ----
void DriverNode::set_joint_zero_service(
  const std::shared_ptr<srvs::SetJointZero::Request> req,
  std::shared_ptr<srvs::SetJointZero::Response> res)
{
  proto::JointConfigCmd cmd;
  cmd.joint_num = req->joint_num;
  cmd.set_zero = true;
  cmd.max_acc = std::numeric_limits<double>::quiet_NaN();  // 不设置加速度

  pending_setjoint_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->zero_set_success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_setjoint_.wait(response_timeout_);
  if (resp) {
    res->success = true;
    res->zero_set_success = resp->zero_set_success;
    res->message = resp->zero_set_success ? "零点设置成功" : "已应答但零点未成功";
  } else {
    res->success = false;
    res->zero_set_success = false;
    res->message = "等待 0x476 应答超时";
  }
}

// ---- 带应答: 清除关节错误 (0x475 clear_err -> 0x476 byte0=0x75) ----
void DriverNode::clear_joint_error_service(
  const std::shared_ptr<srvs::ClearJointError::Request> req,
  std::shared_ptr<srvs::ClearJointError::Response> res)
{
  proto::JointConfigCmd cmd;
  cmd.joint_num = req->joint_num;
  cmd.clear_err = true;
  cmd.max_acc = std::numeric_limits<double>::quiet_NaN();

  pending_setjoint_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_setjoint_.wait(response_timeout_);
  res->success = resp.has_value();
  res->message = resp ? "已清除错误" : "等待 0x476 应答超时";
}

// ---- 带应答: 设置关节加速度 (0x475 max_acc -> 0x476 byte0=0x75) ----
void DriverNode::set_joint_acc_service(
  const std::shared_ptr<srvs::SetJointAcc::Request> req,
  std::shared_ptr<srvs::SetJointAcc::Response> res)
{
  proto::JointConfigCmd cmd;
  cmd.joint_num = req->joint_num;
  cmd.max_acc = req->max_acc;

  pending_setjoint_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_setjoint_.wait(response_timeout_);
  res->success = resp.has_value();
  res->message = resp ? "加速度已设置" : "等待 0x476 应答超时";
}

// ---- 查询: 关节角度/速度限制 (0x472 type=1 -> 0x473) ----
void DriverNode::query_joint_limit_service(
  const std::shared_ptr<srvs::QueryJointLimit::Request> req,
  std::shared_ptr<srvs::QueryJointLimit::Response> res)
{
  proto::JointLimitQueryCmd cmd;
  cmd.joint_num = req->joint_num;
  cmd.query_type = 0x01;

  expected_joint_num_limit_.store(req->joint_num);
  pending_jointlimit_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_jointlimit_.wait(response_timeout_);
  if (resp) {
    res->success = true;
    res->max_angle = static_cast<float>(resp->max_angle);
    res->min_angle = static_cast<float>(resp->min_angle);
    res->max_speed = static_cast<float>(resp->max_speed);
    res->message = "查询成功";
  } else {
    res->success = false;
    res->message = "等待 0x473 应答超时";
  }
}

// ---- 查询: 关节最大加速度 (0x472 type=2 -> 0x47C) ----
void DriverNode::query_joint_max_acc_service(
  const std::shared_ptr<srvs::QueryJointMaxAcc::Request> req,
  std::shared_ptr<srvs::QueryJointMaxAcc::Response> res)
{
  proto::JointLimitQueryCmd cmd;
  cmd.joint_num = req->joint_num;
  cmd.query_type = 0x02;

  expected_joint_num_acc_.store(req->joint_num);
  pending_jointacc_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_jointacc_.wait(response_timeout_);
  if (resp) {
    res->success = true;
    res->max_acc = static_cast<float>(resp->max_acc);
    res->message = "查询成功";
  } else {
    res->success = false;
    res->message = "等待 0x47C 应答超时";
  }
}

// ---- 查询: 末端速度/加速度 (0x477 byte0=1 -> 0x478) ----
void DriverNode::query_end_vel_acc_service(
  const std::shared_ptr<srvs::QueryEndVelAcc::Request> req,
  std::shared_ptr<srvs::QueryEndVelAcc::Response> res)
{
  (void)req;
  proto::ParamQueryCmd cmd;
  cmd.query_type = 0x01;

  pending_endvelacc_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_endvelacc_.wait(response_timeout_);
  if (resp) {
    res->success = true;
    res->max_linear_vel = static_cast<float>(resp->max_linear_vel);
    res->max_angular_vel = static_cast<float>(resp->max_angular_vel);
    res->max_linear_acc = static_cast<float>(resp->max_linear_acc);
    res->max_angular_acc = static_cast<float>(resp->max_angular_acc);
    res->message = "查询成功";
  } else {
    res->success = false;
    res->message = "等待 0x478 应答超时";
  }
}

// ---- 查询: 碰撞防护等级 (0x477 byte0=2 -> 0x47B) ----
void DriverNode::query_collision_level_service(
  const std::shared_ptr<srvs::QueryCollisionLevel::Request> req,
  std::shared_ptr<srvs::QueryCollisionLevel::Response> res)
{
  (void)req;
  proto::ParamQueryCmd cmd;
  cmd.query_type = 0x02;

  pending_collision_.arm();
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }
  const auto resp = pending_collision_.wait(response_timeout_);
  if (resp) {
    res->success = true;
    for (int i = 0; i < 6; ++i) {
      res->level[i] = resp->level[i];
    }
    res->message = "查询成功";
  } else {
    res->success = false;
    res->message = "等待 0x47B 应答超时";
  }
}

// ---- 纯下发: 设置关节限位 (0x474, 协议无应答) ----
void DriverNode::set_joint_limit_service(
  const std::shared_ptr<srvs::SetJointLimit::Request> req,
  std::shared_ptr<srvs::SetJointLimit::Response> res)
{
  proto::JointLimitSetCmd cmd;
  cmd.joint_num = req->joint_num;
  cmd.max_angle = req->max_angle;
  cmd.min_angle = req->min_angle;
  cmd.max_speed = req->max_speed;
  const bool ok = send_frame(encoder_->encode(cmd));
  res->success = ok;
  res->message = ok ? "已下发 (协议无应答, 可用 query_joint_limit 回读确认)" : "CAN 帧发送失败";
}

// ---- 纯下发: 设置末端速度/加速度 (0x479, 协议无应答) ----
void DriverNode::set_end_vel_acc_service(
  const std::shared_ptr<srvs::SetEndVelAcc::Request> req,
  std::shared_ptr<srvs::SetEndVelAcc::Response> res)
{
  proto::EndVelAccSetCmd cmd;
  cmd.max_linear_vel = req->max_linear_vel;
  cmd.max_angular_vel = req->max_angular_vel;
  cmd.max_linear_acc = req->max_linear_acc;
  cmd.max_angular_acc = req->max_angular_acc;
  const bool ok = send_frame(encoder_->encode(cmd));
  res->success = ok;
  res->message = ok ? "已下发 (协议无应答, 可用 query_end_vel_acc 回读确认)" : "CAN 帧发送失败";
}

// ---- 设置碰撞等级 (0x47A); verify=true 时发 0x477(byte0=2) 等 0x47B 回读 ----
void DriverNode::set_collision_level_service(
  const std::shared_ptr<srvs::SetCollisionLevel::Request> req,
  std::shared_ptr<srvs::SetCollisionLevel::Response> res)
{
  proto::CollisionLevelSetCmd cmd;
  for (int i = 0; i < 6; ++i) {
    cmd.level[i] = req->level[i];
  }
  if (!send_frame(encoder_->encode(cmd))) {
    res->success = false;
    res->message = "CAN 帧发送失败";
    return;
  }

  if (!req->verify) {
    res->success = true;
    for (int i = 0; i < 6; ++i) {
      res->applied_level[i] = req->level[i];
    }
    res->message = "已下发 (未校验)";
    return;
  }

  // 主动查询 0x47B 回读
  proto::ParamQueryCmd query;
  query.query_type = 0x02;
  pending_collision_.arm();
  if (!send_frame(encoder_->encode(query))) {
    res->success = false;
    res->message = "已下发但查询帧发送失败";
    return;
  }
  const auto resp = pending_collision_.wait(response_timeout_);
  if (resp) {
    res->success = true;
    for (int i = 0; i < 6; ++i) {
      res->applied_level[i] = resp->level[i];
    }
    res->message = "已下发并回读校验";
  } else {
    res->success = false;
    res->message = "已下发但等待 0x47B 回读超时";
  }
}

// ---- 纯下发: 轨迹/拖动示教控制 (0x150 byte1/byte2) ----
void DriverNode::set_motion_ctrl_service(
  const std::shared_ptr<srvs::SetMotionCtrl::Request> req,
  std::shared_ptr<srvs::SetMotionCtrl::Response> res)
{
  proto::MotionCtrlCmd cmd;
  cmd.trajectory_ctrl = req->trajectory_ctrl;
  cmd.drag_teach = req->drag_teach;
  const bool ok = send_frame(encoder_->encode(cmd));
  res->success = ok;
  res->message = ok ? "已下发" : "CAN 帧发送失败";
}

// ---- 便捷急停 (Trigger; 与 topic ~/stop_cmd 并存) ----
void DriverNode::emergency_stop_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  proto::MotionCtrlCmd cmd;
  cmd.emergency_stop = 0x01;
  const bool ok = send_frame(encoder_->encode(cmd));
  res->success = ok;
  res->message = ok ? "已发送快速急停 (恢复后需重新使能电机)" : "CAN 帧发送失败";
  if (ok) {
    RCLCPP_WARN(get_logger(), "已通过 service 发送快速急停");
  }
}

}  // namespace autorunner_driver
