// CAN 帧 ID 常量与偏移基址计算
// 反馈指令 0x2A1~0x2A8 可整体偏移为 0x2B1~/0x2C1~ (偏移值 0x10/0x20)
// 控制指令 0x150~0x15F 可整体偏移为 0x160~/0x170~ (偏移值 0x10/0x20)
#ifndef AUTORUNNER_DRIVER__PROTOCOL__FRAME_IDS_HPP_
#define AUTORUNNER_DRIVER__PROTOCOL__FRAME_IDS_HPP_

#include <cstdint>

namespace autorunner::protocol
{

// ---- 反馈帧基址 (arm -> PC) ----
constexpr uint32_t kIdArmStatus = 0x2A1;     // 机械臂状态反馈
constexpr uint32_t kIdEndPose1 = 0x2A2;      // 末端位姿反馈1 X/Y
constexpr uint32_t kIdEndPose2 = 0x2A3;      // 末端位姿反馈2 Z/RX
constexpr uint32_t kIdEndPose3 = 0x2A4;      // 末端位姿反馈3 RY/RZ
constexpr uint32_t kIdJointFb12 = 0x2A5;     // 关节反馈 J1/J2
constexpr uint32_t kIdJointFb34 = 0x2A6;     // 关节反馈 J3/J4
constexpr uint32_t kIdJointFb56 = 0x2A7;     // 关节反馈 J5/J6
constexpr uint32_t kIdGripperFb = 0x2A8;     // 夹爪反馈 (本期忽略)

// ---- 控制帧基址 (PC -> arm) ----
constexpr uint32_t kIdMotionCtrl = 0x150;    // 急停/轨迹/拖动示教
constexpr uint32_t kIdModeCtrl = 0x151;      // 模式控制
constexpr uint32_t kIdMoveXy = 0x152;        // 直角坐标指令1 X/Y
constexpr uint32_t kIdMoveZrx = 0x153;       // 旋转坐标指令2 Z/RX
constexpr uint32_t kIdMoveRyrz = 0x154;      // 旋转坐标指令3 RY/RZ
constexpr uint32_t kIdJointCtrl12 = 0x155;   // 关节控制 J1/J2
constexpr uint32_t kIdJointCtrl34 = 0x156;   // 关节控制 J3/J4
constexpr uint32_t kIdJointCtrl56 = 0x157;   // 关节控制 J5/J6
constexpr uint32_t kIdArcPoint = 0x158;      // 圆弧模式坐标序号
constexpr uint32_t kIdGripperCtrl = 0x159;   // 夹爪控制 (本期不用)
constexpr uint32_t kIdMitBase = 0x15A;       // MIT 控制 J1~J6: 0x15A~0x15F

// ---- 配置帧 (无偏移) ----
constexpr uint32_t kIdMotorEnable = 0x471;       // 电机使能/失能
constexpr uint32_t kIdJointLimitQuery = 0x472;   // 查询电机限制
constexpr uint32_t kIdJointLimitFb = 0x473;      // 反馈电机角度/速度限制
constexpr uint32_t kIdJointLimitSet = 0x474;     // 设置电机角度/速度限制
constexpr uint32_t kIdJointConfig = 0x475;       // 关节设置(零点/加速度/清错)
constexpr uint32_t kIdSetResponse = 0x476;       // 设置指令应答
constexpr uint32_t kIdParamQuery = 0x477;        // 参数查询与设置
constexpr uint32_t kIdEndVelAccFb = 0x478;       // 末端速度/加速度反馈
constexpr uint32_t kIdEndVelAccSet = 0x479;      // 末端速度/加速度设置
constexpr uint32_t kIdCollisionSet = 0x47A;      // 碰撞防护等级设置
constexpr uint32_t kIdCollisionFb = 0x47B;       // 碰撞防护等级反馈
constexpr uint32_t kIdJointMaxAccFb = 0x47C;     // 电机最大加速度反馈

// ---- 驱动器反馈 (无偏移) ----
constexpr uint32_t kIdDriverHighBase = 0x251;    // 高速反馈 J1~J6: 0x251~0x256, 5ms
constexpr uint32_t kIdDriverLowBase = 0x261;     // 低速反馈 J1~J6: 0x261~0x266, 100ms

}  // namespace autorunner::protocol

#endif  // AUTORUNNER_DRIVER__PROTOCOL__FRAME_IDS_HPP_
