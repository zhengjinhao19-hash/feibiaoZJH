/**
 *******************************************************************************
 * @file      : motor_device.hpp
 * @brief     : M3508 / M2006 电机设备类与 0x200 控制报文编码
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-12      Hello World     1. 在诊断反馈模型之上补齐控制
 *                                                 指令、方向映射与共享帧编码
 *******************************************************************************
 * @attention :
 *  1. 反馈解析复用 inc/diagnostic_core.hpp 的 MotorFeedback：
 *       - 8 字节反馈帧、0x201~0x204、多圈累计、减速比与方向；
 *       - 该文件仍只负责"读"，本文件负责"写"。
 *  2. 控制帧固定为 0x200，8 字节 = 4 台电机 × int16 大端：
 *       data[0:1] ID1、data[2:3] ID2、data[4:5] ID3、data[6:7] ID4。
 *     三台电机必须在同一帧里一次性发出，不能各自发一帧。
 *  3. setInput() 的输入是"输出轴物理方向"的原始电流指令；encodedInput()
 *     才乘上电机安装方向。这样上层 PID 只面对统一的机构方向，不会把左右
 *     同步带的方向搞反。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_INC_MOTOR_DEVICE_HPP_
#define DART_INC_MOTOR_DEVICE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

#include "diagnostic_core.hpp"

namespace dart
{
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单个电机的静态配置
 * @note  id               CAN 反馈 ID 偏移，反馈帧 = 0x200 + id，控制帧段也按 id 排列
 *        direction        输出轴物理正方向与电调正电流方向的符号（+1 / -1）
 *        reduction_ratio  减速比（M3508: 3591/187，M2006: 36）
 *        raw_limit        控制报文绝对值上限（M3508: 16384，M2006: 10000）
 *        has_temperature  反馈第 6 字节是否可作为温度解释
 */
struct MotorParams {
  uint8_t id = 0;
  int8_t direction = 1;
  float reduction_ratio = 1.0f;
  int16_t raw_limit = 0;
  bool has_temperature = true;
  float current_per_raw = 0.0f;  ///< A/raw，配置用于反馈电流单位换算
};

/**
 * @brief 单台 M3508 / M2006 设备
 * @note  反馈部分不在本类重复实现，而是组合 diagnostic::MotorFeedback；
 *        控制部分只保存一个 int16 原始指令，由 MotorBus 统一打包。
 */
class Motor
{
   public:
    Motor() = default;
    explicit Motor(const MotorParams& params) { init(params); }

    /**
     * @brief 应用静态配置并清空指令
     * @note  配置非法时 configured() 返回 false，所有输出保持 0。
     */
    void init(const MotorParams& params);

    bool configured() const { return configured_; }
    uint8_t id() const { return params_.id; }
    int8_t direction() const { return params_.direction; }
    float reductionRatio() const { return params_.reduction_ratio; }
    int16_t rawLimit() const { return params_.raw_limit; }

    /**
     * @brief 解析一帧 8 字节反馈（由 CAN 接收回调调用）
     * @retval 是否接受该帧
     */
    bool acceptFeedback(const uint8_t* data, size_t length, uint32_t now);

    /**
     * @brief 获取当前反馈样本（相对角度、转速、在线状态等）
     */
    diagnostic::MotorSample sample(uint32_t now) const;

    bool online(uint32_t now) const { return sample(now).online; }

    /**
     * @brief 设置输出轴物理方向上的原始电流指令
     * @param input 期望值，内部按 raw_limit 限幅并四舍五入
     * @note  该接口不会让电机自行转动：调用者必须每周期给出指令，且安全
     *        模式下应调用 resetInput() / setInput(0)。
     */
    void setInput(float input);

    /// 最近一次写入的物理方向指令（已限幅）
    int16_t input() const { return input_raw_; }

    /// 实际写入 0x200 帧的值（已乘方向并再次限幅）
    int16_t encodedInput() const;

    void resetInput() { input_raw_ = 0; }

   private:
    MotorParams params_{};
    diagnostic::MotorFeedback feedback_{1.0f, 1, true};
    int16_t input_raw_ = 0;
    bool configured_ = false;
};

/**
 * @brief 0x200 共享控制帧
 * @note  只保存电机指针，不持有 CAN 句柄；发送动作留在 Task/comm_task.cpp，
 *        便于把"报文组织"和"外设发送"分开测试。
 */
class MotorBus
{
   public:
    static constexpr uint8_t kMaxMotors = 4;
    static constexpr uint16_t kControlId = 0x200;

    /**
     * @brief 按电机 id 挂载（id 必须为 1~4）
     */
    void attach(Motor* motor);

    /**
     * @brief 按 id 取电机，id 越界返回 nullptr
     */
    Motor* motor(uint8_t id) const;

    /**
     * @brief 生成 8 字节 0x200 控制帧
     * @param data 输出缓冲区，至少 8 字节；未挂载的段填 0
     */
    void encode(uint8_t data[8]) const;

   private:
    Motor* motors_[kMaxMotors]{};
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace dart

#endif /* DART_INC_MOTOR_DEVICE_HPP_ */
