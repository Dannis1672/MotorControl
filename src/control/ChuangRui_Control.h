#pragma once
#include "control/MotionControl.h"


using namespace MotionControl;
void Modbus();
void ReadRegister();

class ChuangRui_Control : public Control {
public:
    Result ControlInitial() override;   // 统一用基类名字
    Result ControlFree() override;
    Result WriteBit(UIButton io, bool value) override;
    Result WriteFloat(UIFloat param, float value) override;

    nlohmann::json GetSystemState() override;

    Result AxisMove(Axis axis, float distance) override;
    Result AxisToZero(Axis axis) override;
    Result AxisStop(Axis axis) override;

    bool IsFeed(float zd, float fup) override;
    void ProcessBegin() override;
    void ProcessFinish() override;
};