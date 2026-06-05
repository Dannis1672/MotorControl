#pragma once

#include <atomic>
#include <thread>

#include "json.hpp"
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

 private:
  void Modbus();
  void ReadRegister();
  Result Z_F_Move(float zdistance, float fdistance);
  std::atomic<bool> running_{false};
  std::thread modbus_thread_;
  std::thread read_thread_;
};
