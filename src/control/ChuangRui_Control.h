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

    MotionControl::Result AxisMove(MotionControl::Axis axis, float distance,bool wait = 1) override;
    MotionControl::Result WaitForAxesMoveComplete(MotionControl::Axis axis, float distance);
    //axis移动轴, distance移动距离,wait是否等待，0代表不等待，1代表等待
    Result AxisToZero(MotionControl::Axis axis) override;
    Result AxisStop(MotionControl::Axis axis) override;

    bool IsFeed(float zd, float fup) override;
    void ProcessBegin() override;
    void ProcessFinish() override;

private:
    // 添加这两个线程函数声明
    void Modbus();
    void ReadRegister();

    // 添加线程控制成员变量
    std::atomic<bool> running_{ false };
    std::thread modbus_thread_;
    std::thread read_thread_;


};