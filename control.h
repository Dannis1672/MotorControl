#pragma once
#include<queue>
#include<modbus.h>
#include"json.hpp"

using json = nlohmann::json;


//MB		modbus
//CharmRay	创瑞
//
typedef unsigned __int16 MB_Reg;
typedef  __int16 MB_INT;	//经过确认为有符号数
typedef  __int32 MB_DINT;	//


// CharmRayPlcRegs is defined in control.cpp to keep the header minimal.
//为了实现PLC的接口文档与程序的直接对应，这里使用了‘位域’
//位域的使用详见：https://www.cnblogs.com/Philip-Tell-Truth/p/5805242.html
#pragma pack(push, 2)
struct CharmRayPlcRegs; // forward declaration
#pragma pack(pop)

//todo，位操作宏定义
#define setbit(dest,bit) dest |= (1<<bit) //|=：按位或赋值,将Dest的第bit位置1
#define clrbit(dest,bit) dest &=~(1<<bit) //&=~：按位与非赋值,将dest的第bit位清0


enum Axis {
	Z = 0,
	F,
	C
};



struct FunCode {
	int funCode;//1是灯光，2是电机供电，3加工
	bool value;//0 or 1 ; 或者脉冲数
	float dis;//距离参数, 用于运动控制时指定移动距离
};

struct Rs422 {	//mode（读/写模式）、address（寄存器地址）、number（寄存器数量）、dest（数据缓冲区指针）
	bool mode;//0为读，1为写
	int address;
	int number;
	uint16_t* dest = nullptr;//怎么防止内存泄露
	

	void change(bool Mode, int Address, int Number, uint16_t* Dest) {
		mode = Mode;
		address = Address;
		number = Number;
		dest = Dest;
	}
};



static modbus_t* rtu_var_;


extern CharmRayPlcRegs out_data; // defined in control.cpp
extern CharmRayPlcRegs MyData;  // defined in control.cpp
void Config_innitial();
template<typename T>
void Config_set(const std::string& group, const std::string& key, T& value);
template<typename T>
void Config_set(const std::string& key, const T& value);
void ModbusInitial();
void ModbusTaskPush(Rs422&);

void ReadRegister();
void Z_F_move(float, float);
void Axis_Move(Axis, float);
void Light_switch() {};
void Modbus();

CharmRayPlcRegs& get();
CharmRayPlcRegs* write();
