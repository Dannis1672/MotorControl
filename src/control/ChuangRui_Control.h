#pragma once
#include "control/MotionControl.h"

class ChuangRui_Control : public MotionControl::Control {
public:
	using Result =  MotionControl::Result;

    Result ControlInitial() override;   // 统一用基类名字
    Result ControlFree() override;
    Result WriteBit(MotionControl::UIButton io, bool value) override;
    Result WriteFloat(MotionControl::UIFloat param, float value) override;
    nlohmann::json GetSystemState() override;

    Result AxisMoveAnsyc(MotionControl::Axis axis, float distance) override;
    Result AxisToZero(MotionControl::Axis axis) override;
    Result AxisStop(MotionControl::Axis axis) override;

    bool IsFeed(float zd, float fup) override;
    void ProcessBegin() override;
    void ProcessFinish() override;

private:
    // 添加这两个线程函数声明
    void Modbus();
    void ReadRegister();
    //
    Result AxisMoveAwait(MotionControl::Axis axis, float distance);
    Result WaitForAxesMoveComplete(MotionControl::Axis axis, float distance);
    // 添加线程控制成员变量
    std::atomic<bool> running_{ false };
    std::thread modbus_thread_;
    std::thread read_thread_;


};