#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include "json.hpp"

#define CONTROL_NAMESAPCE_BEGIN namespace MotionControl{
#define CONTROL_NAMESAPCE_END };

CONTROL_NAMESAPCE_BEGIN
enum UIButton {
	//suggested
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


	//unsuggested
	LaserCooler,
	LaserPower,
	LaserIndicate,
	PowderFeed,
	Feed,
	CLeft,
	CRight,
	Fup,
	FDown,
	MotorPower
};

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
	Test
};
enum Axis {
	Z = 0, //成型缸
	F,		//供粉缸
	C		//铺粉辊
};

enum class Result {
	Success = 0,
	Failure
};
class Control
{
public:

	virtual Result ControlInitial() =0;	//成功返回值为零，失败返回值为-1，并抛出异常Warning or Error
	virtual Result ControlFree() = 0;
	virtual Result WriteBit(UIButton io, bool value) = 0;
	virtual Result WriteFloat(UIFloat, float value) = 0;

	//所有关于硬件的变量都可以从json获取，若键不存在，代码报错
	//实时性不强
	virtual nlohmann::json GetSystemState() = 0;


	//异步指令
	virtual Result AxisMove(Axis axis, float distance) = 0;
	virtual Result AxisToZero(Axis axis) = 0;
	virtual Result  AxisStop(Axis axis) = 0;

	//返回true 即代表铺粉结束，可以出光
	virtual bool IsFeed(float zd, float fup) = 0;
	virtual void ProcessBegin() = 0;
	virtual void ProcessFinish() = 0;
};

CONTROL_NAMESAPCE_END
#endif // !CONTROL_H