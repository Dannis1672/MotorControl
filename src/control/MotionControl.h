#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include "json.hpp"

namespace MotionControl {

// UI buttons (keep original identifiers).
enum UIButton {
  // suggested
  Light = 0,
  ReSet,
  Door,
  GasCharge,
  LaserEnabled,
  BackFlush,
  CharmberPressureTest,
  WholePressureTest,
  Ventilate,
  ProcessMode,

  // unsuggested
  LaserCooler,
  LaserPower,
  LaserIndicate,
  PowderFeed,
  Feed,
  CLeft,
  CRight,
  Fup,
  FDown,
  MotorPower,
};

// Floating UI parameters (keep original identifiers).
enum UIFloat {
  CharmberPressure = 0,
  PressureDown,
  OxygenRatio,
  OxygenRatioDown,
  WindSpeed,
  windPressure,
  Temperature,

  OverallLayers,
  CurrnetLayer,
  Test,
};

// Motion axes (keep original identifiers).
enum Axis {
  Z = 0,  // 成型缸
  F,      // 供粉缸
  C       // 铺粉辊
};

// Result of control operations.
enum class Result {
  Success = 0,
  Failure,

};

class Control {
 public:
  virtual ~Control() = default;

  // Initialize control resources. Return Result::Success on success.
  virtual Result ControlInitial() = 0;

  // Release control resources.
  virtual Result ControlFree() = 0;

  virtual Result WriteBit(UIButton io, bool value) = 0;
  virtual Result WriteFloat(UIFloat, float value) = 0;

  // Return a snapshot of the system state as JSON.
  virtual nlohmann::json GetSystemState() = 0;

  // Asynchronous commands.
  virtual Result AxisMove(Axis axis, float distance) = 0;
  virtual Result AxisToZero(Axis axis) = 0;
  virtual Result AxisStop(Axis axis) = 0;

  // Returns true when feeding is complete.
  virtual bool IsFeed(float zd, float fup) = 0;
  virtual void ProcessBegin() = 0;
  virtual void ProcessFinish() = 0;
};

}  // namespace MotionControl

#endif  // MOTIONCONTROL_H
