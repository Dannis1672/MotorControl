#pragma once

#include <atomic>

#include "control/CharmRayPlcRegs.h"
#include "json.hpp"
#include "control/ModbusClient.h"
#include "control/MotionControl.h"

class ChuangRui_Control : public MotionControl::Control {
 public:
  using Result = MotionControl::Result;

  Result ControlInitial() override;
  Result ControlFree() override;


  Result WriteBit(MotionControl::UIButton io, bool value) override;
  Result WriteFloat(MotionControl::UIFloat param, float value) override;
  nlohmann::json GetSystemState() override;
  Result AxisMove(MotionControl::Axis axis, float distance) override;
 

  Result AxisToZero(MotionControl::Axis axis) override;
  Result AxisStop(MotionControl::Axis axis) override;

  bool IsFeed(float zd, float fup) override;
  void ProcessBegin() override;
  void ProcessFinish() override;

#ifdef MOTOCONTROL_TEST
  /// 测试辅助：直接设置 PLC 寄存器快照（用于 GetSystemState 等纯逻辑测试）。
  static void TestSetRegisters(const CharmRayPlcRegs& regs);

  /// 测试辅助：获取运动控制参数（用于验证参数计算逻辑）。
  static float TestGetMotorRatio(int axis);
  static float TestGetMoveDistance(int axis);
  static float TestGetAccelerate(int axis);
  static float TestGetVelocity(int axis);
#endif

 private:
  Result Z_F_Move(float zdistance, float fdistance);
  std::atomic<bool> running_{false};
  ModbusClient modbus_;
};