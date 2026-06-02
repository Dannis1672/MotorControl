#pragma once
#include "MotionControl.h"
#include <modbus.h>
#include <fstream>
#include <iostream>
#include <chrono>

using namespace MotionControl;

class ChaungRui_Control
{
public:
	Result ChaungRui_ControlInitial();	//成功返回值为零，失败返回值为-1，并抛出异常Warning or Error
    Result ControlFree();
	Result WriteBit(UIButton io, bool value) ;
	Result WriteFloat(UIFloat, float value) ;

	//所有关于硬件的变量都可以从json获取，若键不存在，代码报错
	//实时性不强  
	 nlohmann::json GetSystemState() ;


	//异步指令
	Result AxisMove(Axis axis, float distance) ;
	Result AxisToZero(Axis axis) ;
    Result  AxisStop(Axis axis) ;

	//返回true 即代表铺粉结束，可以出光
	bool IsFeed(float zd, float fup) ;
	void ProcessBegin() ;
	void ProcessFinish() ;

};
