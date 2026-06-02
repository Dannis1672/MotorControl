<<<<<<< HEAD
ï»¿#ifndef CONTROL_H
#define CONTROL_H
#include"json.hpp"
=======
#ifndef CONTROL_H
#define CONTROL_H


#include <json.hpp>

>>>>>>> ecdd978d91830908f1c0888dc03e890047d8a964
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
<<<<<<< HEAD
	Z = 0, //æˆåž‹ç¼¸
	F,		//ä¾›ç²‰ç¼¸
	C		//é“ºç²‰è¾Š
};

enum class Result {
	Success = 0,
	Failure
};

=======
	Z = 0, //³ÉÐÍ¸×
	F,		//¹©·Û¸×
	C		//ÆÌ·Û¹õ
};

enum class Result{
	Success =0,
	Failure
};
>>>>>>> ecdd978d91830908f1c0888dc03e890047d8a964
class Control
{
public:

<<<<<<< HEAD
	virtual Result ControlInitial() =0;	//æˆåŠŸè¿”å›žå€¼ä¸ºé›¶ï¼Œå¤±è´¥è¿”å›žå€¼ä¸º-1ï¼Œå¹¶æŠ›å‡ºå¼‚å¸¸Warning or Error
	virtual Result ControlFree() = 0;
	virtual Result WriteBit(UIButton io, bool value) = 0;
	virtual Result WriteFloat(UIFloat, float value) = 0;

	//æ‰€æœ‰å…³äºŽç¡¬ä»¶çš„å˜é‡éƒ½å¯ä»¥ä»ŽjsonèŽ·å–ï¼Œè‹¥é”®ä¸å­˜åœ¨ï¼Œä»£ç æŠ¥é”™
	//å®žæ—¶æ€§ä¸å¼º  
	virtual nlohmann::json GetSystemState() = 0;


	//å¼‚æ­¥æŒ‡ä»¤
=======
	virtual Result ControlInitial() =0;	//³É¹¦·µ»ØÖµÎªÁã£¬Ê§°Ü·µ»ØÖµÎª-1£¬²¢Å×³öÒì³£Warning or Error
	virtual Result ControlFree() = 0;

	virtual Result WriteBit(UIButton io, bool value) = 0;
	virtual Result WriteFloat(UIFloat, float value) = 0;

	//ËùÓÐ¹ØÓÚÓ²¼þµÄ±äÁ¿¶¼¿ÉÒÔ´Ójson»ñÈ¡£¬Èô¼ü²»´æÔÚ£¬´úÂë±¨´í
	//ÊµÊ±ÐÔ²»Ç¿  
	virtual nlohmann::json GetSystemState() = 0;


	//Òì²½Ö¸Áî
>>>>>>> ecdd978d91830908f1c0888dc03e890047d8a964
	virtual Result AxisMove(Axis axis, float distance) = 0;
	virtual Result AxisToZero(Axis axis) = 0;
	virtual Result  AxisStop(Axis axis) = 0;

<<<<<<< HEAD
	//è¿”å›žtrue å³ä»£è¡¨é“ºç²‰ç»“æŸï¼Œå¯ä»¥å‡ºå…‰
	virtual bool IsFeed(float zd, float fup) = 0;
	virtual void ProcessBegin() = 0;
	virtual void ProcessFinish() = 0;

=======
	//·µ»Øtrue ¼´´ú±íÆÌ·Û½áÊø£¬¿ÉÒÔ³ö¹â
	virtual bool IsFeed(float zd,float fup) = 0;
	virtual void ProcessBegin() = 0;
	virtual void ProcessFinish() = 0;
	
>>>>>>> ecdd978d91830908f1c0888dc03e890047d8a964
};

CONTROL_NAMESAPCE_END
#endif // !CONTROL_H