/**
 *******************************************************************************
 * @file      : motor_configs.hpp
 * @brief     : 电机 / 舵机参数与各路控制环 PID 配置
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：提供精简版舵机参数
 *  V1.1.0      2026-11-12      Hello World     2. 补齐 M3508 / M2006 参数、
 *                                                 机械换算、位置环 PID 与
 *                                                 安全/时序阈值
 *******************************************************************************
 * @attention :
 *  1. 方向、减速比来自考核说明 2.1 / 6.2 节；方向最终仍需实车标定。
 *  2. PID 参数是保守初值：先在悬空/低速下验证方向与限幅，再逐步提高 Kp。
 *  3. 所有位置量单位：同步带 mm，丝杆 height（1 height = 2.5 mm 行程）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_CONFIGS_MOTOR_CONFIGS_HPP_
#define DART_CONFIGS_MOTOR_CONFIGS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "base.hpp"
#include "motor_device.hpp"
#include "pid.hpp"
#include "servo.hpp"

namespace hw_servo = hello_world::servo;

/* Exported constants --------------------------------------------------------*/
//* 舵机相关配置（已提供） *//
const hw_servo::ServoInfo kServoShoot{
    .operating_freq = 200.0f,
    .min_pulse_duration = 50,
    .max_pulse_duration = 250,
    .angle_range = 180.0f,
};

namespace dart
{
/* ------------------------------- 机械换算 -------------------------------- */
inline constexpr float kPi = 3.14159265358979323846f;

/// length(mm) = angle(rad) × 16.8；目标角度 = 目标长度 × kLength2Angle
inline constexpr float kAngle2Length = 16.8f;
inline constexpr float kLength2Angle = 1.0f / kAngle2Length;

/// angle(rad) = π × height；height = angle / π
inline constexpr float kHeight2Angle = kPi;
inline constexpr float kAngle2Height = 1.0f / kPi;

/* ------------------------------- 电机参数 -------------------------------- */
// 同步带右：id=1，正向，反馈 0x201，控制段 data[0:1]
inline constexpr MotorParams kMotorBeltRhs{
    1, 1, 3591.0f / 187.0f, 16384, true, 20.0f / 16384.0f};
// 同步带左：id=2，反向，反馈 0x202，控制段 data[2:3]
inline constexpr MotorParams kMotorBeltLhs{
    2, -1, 3591.0f / 187.0f, 16384, true, 20.0f / 16384.0f};
// 丝杆：id=3，反向，反馈 0x203，控制段 data[4:5]
inline constexpr MotorParams kMotorScrew{3, -1, 36.0f, 10000, false, 10.0f / 10000.0f};

/* -------------------------------- PID 参数 -------------------------------- */
// 位置环输入为输出轴角度 rad，输出为电调原始电流指令。
// integral_limit / output_limit 均按 M3508 报文 ±16384 留出余量。
inline constexpr learning::PidConfig kPidBelt{
    5000.0f,  // Kp
    300.0f,   // Ki
    120.0f,   // Kd（对测量值微分，避免目标跳变冲击）
    2500.0f,  // 积分限幅
    8000.0f,  // 输出限幅（约 10 A）
};

// 丝杆 M2006 输出更小，必须比同步带更保守。
inline constexpr learning::PidConfig kPidScrew{
    2500.0f,  // Kp
    600.0f,   // Ki
    80.0f,    // Kd
    1500.0f,  // 积分限幅
    2500.0f,  // 输出限幅（约 2.5 A）
};

/* ------------------------------ 同步带参数 ------------------------------- */
inline constexpr float kBeltPullDownLength = -825.0f;  ///< 拉下位，单位 mm
inline constexpr float kBeltPullUpLength = -5.0f;      ///< 拉起位（新镖架）
inline constexpr float kBeltLengthMin = -832.0f;       ///< 目标下限，禁止再往下
inline constexpr float kBeltLengthMax = 0.0f;          ///< 上限位即机械零点

inline constexpr float kBeltPosTolerance = 3.0f;   ///< 到位窗口，单位 mm
inline constexpr float kBeltEndpointPosToleranceMm = 0.5f; ///< 每侧到位误差，mm
inline constexpr float kBeltEndpointSettleSpeed = 0.15f; ///< 每侧到位低速条件 rad/s
inline constexpr float kBeltMaxSpeed = 35.0f;      ///< 轨迹最大速度，输出轴 rad/s
inline constexpr float kBeltMaxAccel = 90.0f;      ///< 轨迹最大加速度，rad/s²
// Automatic Moving only; set gain to zero to restore pure position PID.
inline constexpr float kBeltVelocityFeedforward = 50.0f; // raw/(rad/s)
inline constexpr float kBeltFeedforwardLimit = 1500.0f; // included in total current limit
inline constexpr float kBeltMaxDecel = 160.0f;    ///< 末端制动减速度，rad/s²
// Only automatic upward moves ending near the top use this approach profile.
inline constexpr bool kBeltReturnApproachEnabled = true;
inline constexpr float kBeltReturnDecel = 80.0f; // earlier, gentler braking rad/s²
inline constexpr float kBeltReturnApproachSpeed = 5.0f; // rad/s
inline constexpr float kBeltReturnApproachMm = 25.0f; // final slow zone before target
inline constexpr uint32_t kBeltMoveTimeoutMs = 7000;  ///< 单次移动超时
inline constexpr uint32_t kBeltSettleMs = 120;        ///< 到位防抖
inline constexpr uint32_t kBeltStartupGraceMs = 500;  ///< 使能后等待首帧反馈

// 双电机同步补偿：以两台物理角度差做 P 修正，差值过大时不再硬顶。
// 首次上板建议先置 0，确认两台角度方向/零点一致后再启用。
inline constexpr float kBeltSyncKp = 600.0f;      ///< raw / rad
inline constexpr float kBeltSyncLimit = 1500.0f;  ///< 同步补偿限幅
// Automatic motion: differential position/velocity feedback, separate from homing/manual.
inline constexpr float kBeltAutoSyncKp = 3000.0f; // raw/rad
inline constexpr float kBeltAutoSyncKd = 300.0f;  // raw/(rad/s), measured velocity
inline constexpr float kBeltAutoSyncLimit = 2000.0f;
inline constexpr float kBeltAutoInputLimit = 8000.0f; // includes synchronization
inline constexpr float kBeltMaxSkewRad = 0.35f;  ///< 两侧相对位移差超过约 5.9 mm 停机
inline constexpr float kBeltArrivalSkewRad = 0.12f;
// Automatic bottom reversal: no dwell; only a low-speed, synchronized endpoint.
inline constexpr float kBeltTurnaroundMaxSpeedRad = 2.0f;
inline constexpr float kBeltTravelMarginMm = 5.0f;

// 独立滑块回零：每侧分别低速寻找机械上止点，两侧确认后重建同步基准。
// 禁用时拒绝建立零点及后续单发，不能用任意当前位置冒充机械零点。
inline constexpr bool kEnableBeltHoming = true;
inline constexpr float kBeltHomingInput = 2000.0f;     ///< 复位电流（raw）
inline constexpr float kBeltHomingSpeedRad = 5.0f;
inline constexpr float kBeltHomingSpeedKp = 800.0f;
inline constexpr float kBeltHomingAccelRad = 10.0f; // ramped approach
inline constexpr float kBeltHomingHoldInput = 1200.0f; // contact side, bounded preload
inline constexpr float kBeltHomingStallSpeed = 0.05f;  ///< 接触候选最大转速，rad/s
inline constexpr uint32_t kBeltHomingStallMs = 500;    ///< 连续稳定接触确认时间
inline constexpr float kBeltHomingContactDriftMm = 0.05f; ///< 确认期间每侧允许位置变化
inline constexpr uint32_t kBeltHomingStartMs = 600;    ///< 启动阶段不判堵转
inline constexpr uint32_t kBeltHomingTimeoutMs = 30000;
inline constexpr float kBeltHomingContactCurrentRaw = 1000.0f;
inline constexpr float kBeltHomingReverseToleranceRad = 0.15f;
inline constexpr float kBeltHomingMaxTravelRad = 52.0f;
// 电流和运动检查仅排除部分故障，不能区分真正上限位与中途机械卡死。

// 运动/保持过程中的堵转保护。只有"误差很大 + 指令很大 + 几乎不转"才判堵转，
// 避免正常到位保持被误判。
inline constexpr float kBeltStallError = 10.0f;    ///< mm
inline constexpr float kBeltStallInput = 3000.0f;  ///< raw
inline constexpr float kBeltStallSpeed = 1.0f;     ///< rad/s
inline constexpr uint32_t kBeltStallMs = 500;

// Operator-requested unloading; communication/device faults still stop immediately.
inline constexpr float kBeltReliefTargetMm = -5.0f;
// Automatic pulldown: either slide entering this region may already have loaded.
// Operator release must finish lowering before opening the trigger and returning.
inline constexpr float kBeltLoadedReliefThresholdMm = -750.0f;
static_assert(kBeltLoadedReliefThresholdMm >= kBeltPullDownLength &&
              kBeltLoadedReliefThresholdMm < kBeltPullUpLength,
              "Loaded relief threshold must lie between the bottom and upper targets");
inline constexpr float kBeltReliefBrakeDecel = 160.0f; // per-side speed ramp rad/s²
inline constexpr float kBeltReliefSpeedRad = 5.0f;
inline constexpr float kBeltReliefAccel = 20.0f;
inline constexpr float kBeltReliefDecel = 40.0f;
inline constexpr float kBeltReliefPositionKp = 8.0f; // rad/s per rad
inline constexpr float kBeltReliefSettleSpeed = 0.15f;
inline constexpr float kBeltReliefMaxActualSpeed = 8.0f;
inline constexpr uint32_t kBeltReliefBrakeSettleMs = 80;
inline constexpr uint32_t kBeltReliefSettleMs = 200;
inline constexpr uint32_t kBeltReliefTimeoutMs = 40000;
inline constexpr uint32_t kReliefTriggerHoldMs = 1000; // after actual 180-degree PWM write
inline constexpr learning::PidConfig kPidBeltReliefSpeed{800,800,0,8000,8000};

/* ------------------------------- 手动参数 -------------------------------- */
// 手动档：左下 + 右上；左竖轴升降，右竖轴上推击发。
inline constexpr float kManualStickDeadband = 0.10f;
inline constexpr float kManualFireThreshold = 0.80f;
inline constexpr float kManualFireMaxSpeedRad = 0.10f;
inline constexpr uint32_t kManualFireSettleMs = 120;
inline constexpr float kManualBeltMaxSpeedRad = 2.0f;  ///< 约 33.6 mm/s
inline constexpr float kManualBeltMaxAccelRad = 10.0f;
inline constexpr float kManualBeltSpeedKp = 800.0f;
inline constexpr float kManualBeltInputLimit = 2000.0f;
inline constexpr float kManualBeltSyncLimit = 500.0f;
inline constexpr float kManualBeltMaxActualSpeedRad = 5.0f;
inline constexpr float kManualBeltBrakeDistanceMm = 10.0f;
inline constexpr float kManualBeltLimitMarginMm = 1.0f;
inline constexpr float kManualBeltStallInput = 1000.0f;
inline constexpr float kManualBeltStallSpeed = 0.15f;
inline constexpr uint32_t kManualBeltStallMs = 500;

/* ------------------------------- 丝杆参数 -------------------------------- */
inline constexpr float kScrewTravelLimit = 2.0f;  ///< 相对零点行程，height
inline constexpr float kScrewTrimStep = 0.01f;    ///< 单次微调步长，height
inline constexpr float kScrewHoldDeadbandRad = 0.015f;
inline constexpr float kScrewSettleSpeed = 0.3f;   ///< rad/s（输出轴）
inline constexpr float kScrewStallErrorRad = 0.15f;
inline constexpr float kScrewStallSpeed = 0.3f;
inline constexpr uint32_t kScrewStallMs = 500;
inline constexpr uint32_t kScrewStartupGraceMs = 500;
inline constexpr float kScrewTargetSpeedHeight = 0.2f;
inline constexpr float kScrewMaxSpeedRad = 3.0f;

/* ------------------------------- 击发参数 -------------------------------- */
inline constexpr float kShootReadyAngle = 20.0f;  ///< 扳机复位（安全角）
inline constexpr float kShootFireAngle = 95.0f;  ///< 扳机下拉（击发）

/* ------------------------------ 顶层时序参数 ------------------------------ */
inline constexpr float kInnerEightThreshold = 0.8f;
inline constexpr uint32_t kShootHoldMs = 1000;     ///< 180° 保持时间
inline constexpr uint32_t kShotIntervalMs = 500;   ///< 两发最小间隔
// Open-loop allowance after the 95-degree command is actually written.
inline constexpr uint32_t kServoResetSettleMs = 500;
inline constexpr uint32_t kShotFinishMs = kServoResetSettleMs; ///< Finish 最短等待
inline constexpr uint32_t kHomingStepTimeoutMs = kBeltHomingTimeoutMs + kBeltMoveTimeoutMs + 1000;
inline constexpr uint32_t kMoveStepTimeoutMs = 10000;
inline constexpr uint32_t kShootStepTimeoutMs = 3000;
inline constexpr uint32_t kFinishStepTimeoutMs = kServoResetSettleMs + 1000;

// 当前用户要求：自动松开内八/切安全档执行受控卸力；设备/通信故障仍立即停止输出。

}  // namespace dart

#endif /* DART_CONFIGS_MOTOR_CONFIGS_HPP_ */
