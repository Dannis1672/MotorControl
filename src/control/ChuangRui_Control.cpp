#include<chrono>
#include<mutex>
#include<shared_mutex>
#include<thread>
#include<condition_variable>
#include<fstream>
#include<iostream>
#include<queue>
#include <spdlog/spdlog.h>
#include<modbus.h>
#include"json.hpp"
#include"control/MotionControl.h"
#include"control/ChuangRui_Control.h"


using namespace std;
using namespace MotionControl;

using json = nlohmann::json;

struct Rs422 {	//mode（读/写模式）、address（寄存器地址）、number（寄存器数量）、dest（数据缓冲区指针）
	bool mode=0;//0为读，1为写
	int address=0;
	int number =1;
	uint16_t* dest = nullptr;//怎么防止内存泄露
	Rs422(bool mode, int address, int number, uint16_t* dest) :mode(mode), address(address), number(number), dest(dest) {

	}
	Rs422() = default;
	void change(bool Mode, int Address, int Number, uint16_t* Dest) {
		mode = Mode;
		address = Address;
		number = Number;
		dest = Dest;
	}
};
mutex modbus_mtx;
shared_mutex mutex_;
condition_variable queueCondVar;
condition_variable ModbusQueueCondVar;
std::queue<Rs422>modbus_task;

json global_config;
static modbus_t* rtu_var_;

//运动控制参数定义
static float Z_Move_Distance = 0.5f;
static float Z_Accelerate = 12.0f;
static float Z_Velocity = 3.0f;

static float F_Move_Distance = 1.0f;
static float F_Accelerate = 12.0f;
static float F_Velocity = 3.0f;

static float C_Move_Distance = 20.0f;
static float C_Accelerate = 350.0f;
static float C_Velocity = 150.0f;

static struct {
	char device[32] = "COM2";//串行端口名称
	int baud = 19200;//波特率
	char parity = 'N';//奇偶校验
	int data_bit = 8;//指定数据的位数
	int stop_bit = 1;//指定停止位位数
}rtu_set_;
#pragma pack(push, 2)
struct CharmRayPlcRegs
{
	//Modbus地址40011
	struct MW10
	{
		unsigned short security : 1;        //0
		unsigned short light : 1;            //1
		unsigned short red_light : 1;        //2
		unsigned short greeen_light : 1;    //3
		unsigned short blue_light : 1;        //4
		unsigned short alarm : 1;            //5
		unsigned short big_valve : 1;        //6
		unsigned short small_valve : 1;    //7
		unsigned short ventilate : 1;        //8
		unsigned short lase_power : 1;        //9
		unsigned short fan_power : 1;        //10
		unsigned short motor_power : 1;        //11
		unsigned short laser_indicator : 1;    //12
		unsigned short laser_enabled : 1;    //13
		unsigned short laser_on : 1;        //14
		unsigned short fan_on : 1;            //15

	}MW10;

	//Modbus地址40012
	struct MW11
	{
		unsigned short Z轴回原点指令 : 1;        //0
		unsigned short F轴回原点指令 : 1;        //1
		unsigned short C轴回原点指令 : 1;        //2
		unsigned short Z轴运动开始指令 : 1;        //3
		unsigned short F轴运动开始指令 : 1;        //4
		unsigned short C轴运动开始指令 : 1;        //5
		unsigned short unused_6_15 : 10;        //6-15, 共10位
	}MW11;

	//Modbus地址40013
	struct MW13_MW14
	{
		//MW13
		unsigned short 大充气阀状态 : 1;        //0
		unsigned short 小充气阀状态 : 1;        //1
		unsigned short 排气阀状态 : 1;            //2
		unsigned short 急停安全继电器状态 : 1;    //3
		unsigned short 腔门插到位信号 : 1;        //4
		unsigned short 腔门锁上电状态 : 1;        //5
		unsigned short 激光器有电信号 : 1;        //6
		unsigned short 风机有电信号 : 1;            //7
		unsigned short 电机有电信号 : 1;        //8
		//MW14    
		unsigned short Z轴原点标志 : 1;            //9
		unsigned short F轴原点标志 : 1;            //10
		unsigned short C轴原点标志 : 1;            //11
		//
		unsigned short unused_12_15 : 4;        //第12位至第15位，共计4位
	}MW13_MW14;

	//Modbus地址40014
	struct MW12
	{
		unsigned short 激光器报警 : 1;            //0
		unsigned short 风机报警 : 1;            //1
		unsigned short unused_2_15 : 14;        //第2位到第15位，共计14位
	}MW12;


	//Modbus地址40015
	struct MW12_12_14
	{
		unsigned short Z轴驱动器报警 : 1;        //0
		unsigned short Z轴运动超时报警 : 1;        //1
		unsigned short F轴驱动器报警 : 1;        //2
		unsigned short F轴运动超时报警 : 1;        //3
		unsigned short C轴驱动器报警 : 1;        //4
		unsigned short C轴运动超时报警 : 1;        //5
		unsigned short Z轴正限位 : 1;            //6
		unsigned short F轴正限位 : 1;            //7
		unsigned short C轴正限位 : 1;            //8
		unsigned short Z轴负限位 : 1;            //9
		unsigned short F轴负限位 : 1;            //10
		unsigned short C轴负限位 : 1;            //11
		unsigned short Z轴运动结束信号 : 1;        //12
		unsigned short F轴运动结束信号 : 1;        //13
		unsigned short C轴运动结束信号 : 1;            //14
		unsigned short 铺粉到位 : 1;                //15

	}MW12_12_14;

	//Modbus地址40016
	struct STOP{
		unsigned short Z轴运动停止信号 : 1;        //0
		unsigned short F轴运动停止信号 : 1;        //1
		unsigned short C轴运动停止信号 : 1;        //2
		unsigned short unused_3_15 : 13;        //第3位到第15位，共计13位
	}STOP;
	//Mobus地址40017-40020
	uint16_t unused_17_20[4];

	//Mobus地址40021-40038
	int32_t Z轴加速度;
	int32_t Z轴运动速度;
	int32_t Z轴移动距离;

	int32_t F轴加速度;
	int32_t F轴运动速度;
	int32_t F轴移动距离;

	int32_t C轴加速度;
	int32_t C轴运动速度;
	int32_t C轴移动距离;


	//Modbus地址40039-40040
	int16_t unused_39_40[2];

	//Modbus地址40041-40043
	int16_t 风压设定值;
	int16_t 压力预警设定值;
	int16_t 压力报警设定值;

	//Modbus地址40044-40050
	uint16_t usused44_50[7];

	//Modbus地址40051-40054
	int16_t 氧含量低精度;
	int16_t 氧含量高精度;
	int16_t 风压实际值;
	int16_t 腔体压力;

	//Modbus地址40055-40060
	uint16_t usused55_60[6];

	//Modbus地址40061-40066
	int32_t Z轴当前位置;
	int32_t F轴当前位置;
	int32_t C轴当前位置;

};
#pragma pack(pop)

static CharmRayPlcRegs out_data;
static CharmRayPlcRegs MyData;
static float Motor_ratio[3] = { 5000.0f, 5000.0f, 95.2f };


// JSON 读取辅助：key 缺失时记录 warn 日志，返回 fallback
template<typename T>
T config_get(const nlohmann::json& j, const std::string& key, T fallback) {
	if (!j.contains(key)) {
		spdlog::warn("config.json: key \"{}\" not found, using fallback", key);
		return fallback;
	}
	return j[key].get<T>();
}

// 嵌套 JSON 读取辅助：用于 "parent.child" 两层结构
template<typename T>
T config_get_nested(const nlohmann::json& j,
                    const std::string& parent,
                    const std::string& child,
                    T fallback) {
	if (!j.contains(parent)) {
		spdlog::warn("config.json: section \"{}\" not found, using fallback for \"{}\"", parent, child);
		return fallback;
	}
	if (!j[parent].contains(child)) {
		spdlog::warn("config.json: key \"{}.{}\" not found, using fallback", parent, child);
		
		return fallback;
	}
	return j[parent][child].get<T>();
}


Result ChuangRui_Control::ControlInitial()//初始化函数
{
	std::ifstream file("config.json");
	if (!file.is_open()) {
		spdlog::warn("config.json not found, using default configuration");
		global_config = json::object();
	} else {
		try {
			global_config = json::parse(file);
			file.close();
		} catch (const json::parse_error& e) {
			spdlog::error("config.json parse error: {}, using default configuration", e.what());
			global_config = json::object();
		}
	}

	// Initialize MyData and out_data from configuration if keys exist.
	// For array-like registers the config uses a scalar default; fill all elements with that value.


	MyData.Z轴加速度 = config_get(global_config, "Z轴加速度", 0);
	MyData.Z轴运动速度 = config_get(global_config, "Z轴运动速度", 0);
	MyData.Z轴移动距离 = config_get(global_config, "Z轴移动距离", 0);

	MyData.F轴加速度 = config_get(global_config, "F轴加速度", 0);
	MyData.F轴运动速度 = config_get(global_config, "F轴运动速度", 0);
	MyData.F轴移动距离 = config_get(global_config, "F轴移动距离", 0);

	MyData.C轴加速度 = config_get(global_config, "C轴加速度", 0);
	MyData.C轴运动速度 = config_get(global_config, "C轴运动速度", 0);
	MyData.C轴移动距离 = config_get(global_config, "C轴移动距离", 0);

	MyData.风压设定值 = config_get(global_config, "风压设定值", 0);
	MyData.压力预警设定值 = config_get(global_config, "压力预警设定值", 0);
	MyData.压力报警设定值 = config_get(global_config, "压力报警设定值", 0);

	MyData.氧含量低精度 = config_get(global_config, "氧含量低精度", 0);
	MyData.氧含量高精度 = config_get(global_config, "氧含量高精度", 0);
	MyData.风压实际值 = config_get(global_config, "风压实际值", 0);
	MyData.腔体压力 = config_get(global_config, "腔体压力", 0);

	MyData.Z轴当前位置 = config_get(global_config, "Z轴当前位置", 0);
	MyData.F轴当前位置 = config_get(global_config, "F轴当前位置", 0);
	MyData.C轴当前位置 = config_get(global_config, "C轴当前位置", 0);
	//初始化rtu_set_结构体
	auto device_str = config_get_nested<std::string>(global_config, "rtu_set_", "device", "COM2"); device_str.copy(rtu_set_.device, sizeof(rtu_set_.device) - 1);//串行端口名称
	rtu_set_.device[sizeof(rtu_set_.device) - 1] = '\0';//末尾置0

	rtu_set_.baud = config_get_nested(global_config, "rtu_set_", "baud", 19200);//波特率
	rtu_set_.parity = config_get_nested<std::string>(global_config, "rtu_set_", "parity", "N").at(0);
	rtu_set_.data_bit = config_get_nested(global_config, "rtu_set_", "data_bit", 8);//指定数据的位数
	rtu_set_.stop_bit = config_get_nested(global_config, "rtu_set_", "stop_bit", 1);//指定停止位位数



	//modbus连接,创建一个 Modbus RTU 上下文（对象）,rtu_var_ 是一个 modbus_t* 类型的指针，后续所有操作都通过它来进行。
	rtu_var_ = modbus_new_rtu(rtu_set_.device, rtu_set_.baud, rtu_set_.parity, rtu_set_.data_bit, rtu_set_.stop_bit);

	//设置从站
	int r = modbus_set_slave(rtu_var_, 1);
	if (r == -1) { spdlog::error("Modbus_set_slave error");}

	//打开串口，建立物理连接
	r = modbus_connect(rtu_var_);
	if (r == -1) { spdlog::error("Modbus_connect error"); }

	//主站发送完请求后，等待从站回复的最长时间 100 毫秒
	modbus_set_byte_timeout(rtu_var_, 0, 100000);
	modbus_set_response_timeout(rtu_var_, 0, 100000);
	//这行是在准备要写入的数据
	unsigned int initial = 0;
	r = modbus_write_registers(rtu_var_, 10, 2, (uint16_t*)&initial);//用指针强转16位
	if (r == -1) { spdlog::error("Initial error"); }

	//从配置文件读取运动控制参数	
	Z_Move_Distance = config_get(global_config, "Z_Move_Distance", 0.5f);
	Z_Accelerate = config_get(global_config, "Z_Accelerate", 12.0f);
	Z_Velocity = config_get(global_config, "Z_Velocity", 3.0f);

	F_Move_Distance = config_get(global_config, "F_Move_Distance", 1.0f);
	F_Accelerate = config_get(global_config, "F_Accelerate", 12.0f);
	F_Velocity = config_get(global_config, "F_Velocity", 3.0f);

	C_Move_Distance = config_get(global_config, "C_Move_Distance", 20.0f);
	C_Accelerate = config_get(global_config, "C_Accelerate", 350.0f);
	C_Velocity = config_get(global_config, "C_Velocity", 150.0f);

	out_data.Z轴加速度 = Z_Accelerate * Motor_ratio[Z];
	out_data.Z轴运动速度 = Z_Velocity * Motor_ratio[Z];
	out_data.Z轴移动距离 = Z_Move_Distance * Motor_ratio[Z];

	out_data.F轴加速度 = F_Accelerate * Motor_ratio[F];
	out_data.F轴运动速度 = F_Velocity * Motor_ratio[F];
	out_data.F轴移动距离 = F_Move_Distance * Motor_ratio[F];

	out_data.C轴加速度 = C_Accelerate * Motor_ratio[C];
	out_data.C轴运动速度 = C_Velocity * Motor_ratio[C];
	out_data.C轴移动距离 = C_Move_Distance * Motor_ratio[C];

	modbus_write_registers(rtu_var_, 20, 18, (uint16_t*)&out_data.Z轴加速度);
	// rtu_var_：指向之前创建的 RTU 连接的指针
	// 20：起始寄存器地址
	// 18：连续写 18 个 16 位寄存器
	// out_data.Z轴加速度：在内存中必须紧挨着后面的Z轴运动速度、Z轴移动距离……C轴移动距离
	// 这 9 个成员的数据类型必须是 float（32 位），每个占 2 个寄存器

	running_ = true;
	modbus_thread_ = std::thread(&ChuangRui_Control::Modbus, this);
	read_thread_ = std::thread(&ChuangRui_Control::ReadRegister, this);
	 /*
	 std::thread(...) - 创建一个线程对象
     &ChuangRui_Control::ModbusWorker - 指定要在线程中执行的成员函数
     this - 传递当前对象的指针，让线程知道是哪个对象的函数
     modbus_thread_ - 保存这个线程对象，后续可以管理它（如等待结束）
     */
	return Result::Success;
}

Result ChuangRui_Control::ControlFree() {
	running_ = false;
	ModbusQueueCondVar.notify_all();  // 唤醒Modbus线程
	if (modbus_thread_.joinable()) modbus_thread_.join();
	if (read_thread_.joinable()) read_thread_.join();
	return Result::Success; };

void ModbusTaskPush(Rs422& rs422) {//把一个 Modbus 通信任务放入队列，并通知消费者线程来处理
	{
		lock_guard<mutex>lg2(modbus_mtx);
		modbus_task.push(rs422);
	}
	ModbusQueueCondVar.notify_one();
}

//修改：读写函数，防止80ms死锁程序
void ChuangRui_Control::ReadRegister() {
	while (true) {
		int read_interval = config_get(global_config, "ReadInterval", 200);
		this_thread::sleep_for(chrono::milliseconds(read_interval));

		// 直接在锁内创建任务，使用数据副本
		{
			unique_lock<shared_mutex> lck(mutex_);//锁
			Rs422 rs422;
			// 拷贝一份数据到临时缓冲区
			uint16_t temp_buffer[56];
			memcpy(temp_buffer, &MyData, sizeof(MyData));
			rs422.change(0, 10, 56, temp_buffer);
			ModbusTaskPush(rs422);
		}
	}
}


Result ChuangRui_Control::AxisMoveAwait(Axis axis, float distance) {//单轴运动
	Rs422 rs422;

	//回零
	if (abs(distance) < 1e-6) {
		*(uint16_t*)&out_data.MW11 |= (1 << axis);
	}
	//开始运动
	else
	{
		*(&out_data.Z轴移动距离 + axis * 3) = distance * Motor_ratio[axis];
		rs422.change(1, 24 + 6 * axis, 2, (uint16_t*)(&out_data.Z轴移动距离 + 3 * axis));
		ModbusTaskPush(rs422);
		*(uint16_t*)&out_data.MW11 |= (1 << (axis + 3));//3为bit值偏移量
	}

	rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
	ModbusTaskPush(rs422);
	WaitForAxesMoveComplete(axis, distance);
	return Result::Success;
}


Result ChuangRui_Control::WaitForAxesMoveComplete(Axis axis, float distance){
	//if (wait) return Result::Success;
	Rs422 rs422;
	int count = 0;
	int velocity = 1;  //当前运动轴速度
	switch (axis) {
	case Z: velocity = Z_Velocity; break;
	case F: velocity = F_Velocity; break;
	case C: velocity = C_Velocity; break;
	}

	int timeout = (abs(distance) / velocity*1000 + 2000) * 1000;//单位为ms
	timeout = max(timeout, 600);

	while (count < timeout/10) {
		count++;	//第一次sleep时间大一些
		if (count == 1)
			this_thread::sleep_for(chrono::milliseconds(config_get(global_config, "First_time_delay", 500)));
		else
			this_thread::sleep_for(chrono::milliseconds(config_get(global_config, "Normal_delay", 10)));//读到旧值很可怕
		bool moveStopFlag = false;
		{
			shared_lock<shared_mutex> lck(mutex_);
			moveStopFlag = (*(uint16_t*)(&MyData.MW12_12_14) & (1 << (12 + axis))) >> (12 + axis);
		}
		if (moveStopFlag == 1) {
			// 回零模式：清除回零指令位
			if (abs(distance) < 1e-6) {
				*(uint16_t*)&out_data.MW11 &= (~(1 << axis));
			}
			// 运动模式：清除运动开始指令位
			else
			{
				*(uint16_t*)&out_data.MW11 &= (~(1 << (axis + 3)));//3为bit值偏移量
			}
			rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
			ModbusTaskPush(rs422);

			break;
		}
	}
	if (count == timeout) {
		spdlog::warn("AxisMove timeout, axis={}", static_cast<int>(axis));
		return Result::Failure;
	}
	return Result::Success;

}

Result ChuangRui_Control::AxisToZero(Axis axis) {	// 设置回原点指令位
	Rs422 rs422;
	*(uint16_t*)&out_data.MW11 |= (1 << axis);
	rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
	ModbusTaskPush(rs422);

	// 等待回零完成
	int count = 0;
	while (count < config_get(global_config, "Long_delay", 6000)) {
		count++;
		if (count == 1)
			this_thread::sleep_for(chrono::milliseconds(config_get(global_config, "First_time_delay", 500)));
		else
			this_thread::sleep_for(chrono::milliseconds(config_get(global_config, "Normal_delay", 10)));
		bool stopFlag = (*(uint16_t*)&MyData.MW12_12_14 & (1 << (12 + axis))) >> (12 + axis);
		if (stopFlag) {
			*(uint16_t*)&out_data.MW11 &= ~(1 << axis);
			rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
			ModbusTaskPush(rs422);
		
		}
	}
	if (count == 6000) {
		spdlog::warn("AxisToZero timeout, axis={}", static_cast<int>(axis));
		return Result::Failure;
	}

	return Result::Success;

}

Result  ChuangRui_Control::AxisStop(Axis axis) {	// 清除运动开始指令位,使运动停止
	Rs422 rs422;
	*(uint16_t*)&out_data.MW11 &= ~(1 << (axis + 3));
	rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
	ModbusTaskPush(rs422);
	return Result::Success;
}


Result ChuangRui_Control::WriteBit(UIButton io, bool value) {//设置对应参数状态位，value 为 true → 写 1，false → 写 0
	Rs422 rs422;
	switch (io) {
		// 0: Light
	case UIButton::Light:
		out_data.MW10.light = value ? 1 : 0;  // value 为 true → 写 1，false → 写 0
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

		// 1: ReSet
	case UIButton::ReSet:
		out_data.MW10.security = value ? 1 : 0;
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

		// 2: Door 
	case UIButton::Door:
		out_data.MW13_MW14.腔门锁上电状态 = value ? 1 : 0;
		rs422.change(1, 12, 1, (uint16_t*)&out_data.MW13_MW14);
		break;

		// 3: GasCharge
	case UIButton::GasCharge:
		out_data.MW10.big_valve = value ? 1 : 0;
		out_data.MW10.small_valve = value ? 1 : 0;
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

		// 4: LaserEnabled
	case UIButton::LaserEnabled:
		out_data.MW10.laser_enabled = value ? 1 : 0;
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

		// 5: BackFlush
	case UIButton::BackFlush:
		// 反吹功能（需确认对应寄存器位）
		break;
		// 6: 腔体压力测试（可能是流程控制）
	case UIButton::CharmberPressureTest:
		break;

		// 7: WholePressureTest
	case UIButton::WholePressureTest:
		// 整机压力测试
		break;

		// 8: Ventilate
	case UIButton::Ventilate:
		out_data.MW10.ventilate = value ? 1 : 0;
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

		// 9; 加工模式切换
	case UIButton::ProcessMode:
		break;

		// 10: 激光冷却器控制
	case UIButton::LaserCooler:
		break;

		// 11: LaserPower 
	case UIButton::LaserPower:
		out_data.MW10.lase_power = value ? 1 : 0;
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

		// 12: LaserIndicate 
	case UIButton::LaserIndicate:
		out_data.MW10.laser_indicator = value ? 1 : 0;
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

		// 13: 铺粉控制
	case UIButton::PowderFeed:
		break;

		// 14: 送粉控制
	case UIButton::Feed:
		break;

		// 15: CLeft 
	case UIButton::CLeft:
		if (value) AxisMoveAwait(Axis::C, -C_Move_Distance);
		return Result::Success;

		// 16: CRight 
	case UIButton::CRight:
		if (value) AxisMoveAwait(Axis::C, C_Move_Distance);
		return Result::Success;

		// 17: Fup 
	case UIButton::Fup:
		if (value) AxisMoveAwait(Axis::F, F_Move_Distance);
		return Result::Success;

		//  18: FDown 
	case UIButton::FDown:
		if (value) AxisMoveAwait(Axis::F, -F_Move_Distance);
		return Result::Success;

		//  19: MotorPower
	case UIButton::MotorPower:
		out_data.MW10.motor_power = value ? 1 : 0;
		rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
		break;

	default:
		spdlog::warn("WriteBit: 未知按钮 {}", static_cast<int>(io));
		return Result::Failure;
	}

	ModbusTaskPush(rs422);

	return Result::Success;
};
Result ChuangRui_Control::WriteFloat(UIFloat param, float value) {//环境参数上下限安全设定
	Rs422 rs422;

	switch (param) {
		//  0: CharmberPressure
	case UIFloat::CharmberPressure:
		out_data.腔体压力 = static_cast<int16_t>(value);
		rs422.change(1, 54, 1, (uint16_t*)&out_data.腔体压力);
		break;

		//  1: PressureDown
	case UIFloat::PressureDown:
		out_data.压力预警设定值 = static_cast<int16_t>(value);
		rs422.change(1, 42, 1, (uint16_t*)&out_data.压力预警设定值);
		break;

		//  2: OxygenRatio
	case UIFloat::OxygenRatio:
		out_data.氧含量高精度 = static_cast<int16_t>(value);
		rs422.change(1, 52, 1, (uint16_t*)&out_data.氧含量高精度);
		break;

		//  3: OxygenRatioDown
	case UIFloat::OxygenRatioDown:
		out_data.氧含量低精度 = static_cast<int16_t>(value);
		rs422.change(1, 51, 1, (uint16_t*)&out_data.氧含量低精度);
		break;

		//  4: WindSpeed
	case UIFloat::WindSpeed:
		break;

		//  5: windPressure
	case UIFloat::windPressure:
		out_data.风压设定值 = static_cast<int16_t>(value);
		rs422.change(1, 41, 1, (uint16_t*)&out_data.风压设定值);
		break;

		//  6: Temperature
	case UIFloat::Temperature:
		break;

		//  7: OverallLayers
	case UIFloat::OverallLayers:
		global_config["OverallLayers"] = static_cast<int>(value);
		return Result::Success;

		//  8: CurrnetLayer
	case UIFloat::CurrnetLayer:
		global_config["CurrentLayer"] = static_cast<int>(value);
		return Result::Success;

		//  9: Test
	case UIFloat::Test:
		spdlog::debug("WriteFloat Test: {}", value);
		return Result::Success;

	default:
		return Result::Failure;
	}

	ModbusTaskPush(rs422);

	return Result::Success;
};

bool ChuangRui_Control::IsFeed(float zd, float fup) {

	AxisMoveAwait(F, 0.04);//粉缸上升
	AxisMoveAwait(Z,-0.04);//成型缸避让
	WaitForAxesMoveComplete(F, 0.04);//等待粉缸上升完成	
	WaitForAxesMoveComplete(Z, -0.04);//等待成型缸避让完成	
	AxisMoveAwait(C, 300 );//铺粉
	AxisToZero(C);//回零
	AxisMoveAwait(Z, 0.04);//成型缸回升


	spdlog::info("铺粉完成");
	return true;
};
void ChuangRui_Control::ProcessBegin() {//C轴回零实现
	Rs422 rs422;

	spdlog::info("加工开始");
	//打开激光使能
	out_data.MW10.lase_power = 1;
	//打开电机电源
	out_data.MW10.motor_power = 1;
	//发送
	rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
	ModbusTaskPush(rs422);

};
void ChuangRui_Control::ProcessFinish() {
	spdlog::info("加工结束");
	// 关闭激光
	out_data.MW10.laser_on = 0;
	//关闭使能
	out_data.MW10.lase_power = 0;
	// 停止所有轴运动
	*(uint16_t*)&out_data.MW11 &= ~0x3F;  // 清除 bit 0-5
	// 关闭电机
	out_data.MW10.motor_power = 0;
	// 发送任务
	Rs422 rs422;
	rs422.change(1, 10, 1, (uint16_t*)&out_data.MW10);
	ModbusTaskPush(rs422);
	rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
	ModbusTaskPush(rs422);

};

nlohmann::json ChuangRui_Control::GetSystemState() {
	shared_lock<shared_mutex> lck(mutex_);	// 加锁读取 MyData
	json state;

	// ===== MW10 =====
	state["MW10"]["security"] = static_cast<int>(MyData.MW10.security);
	state["MW10"]["light"] = static_cast<int>(MyData.MW10.light);
	state["MW10"]["red_light"] = static_cast<int>(MyData.MW10.red_light);
	state["MW10"]["green_light"] = static_cast<int>(MyData.MW10.greeen_light);
	state["MW10"]["blue_light"] = static_cast<int>(MyData.MW10.blue_light);
	state["MW10"]["alarm"] = static_cast<int>(MyData.MW10.alarm);
	state["MW10"]["big_valve"] = static_cast<int>(MyData.MW10.big_valve);
	state["MW10"]["small_valve"] = static_cast<int>(MyData.MW10.small_valve);
	state["MW10"]["ventilate"] = static_cast<int>(MyData.MW10.ventilate);
	state["MW10"]["lase_power"] = static_cast<int>(MyData.MW10.lase_power);
	state["MW10"]["fan_power"] = static_cast<int>(MyData.MW10.fan_power);
	state["MW10"]["motor_power"] = static_cast<int>(MyData.MW10.motor_power);
	state["MW10"]["laser_indicator"] = static_cast<int>(MyData.MW10.laser_indicator);
	state["MW10"]["laser_enabled"] = static_cast<int>(MyData.MW10.laser_enabled);
	state["MW10"]["laser_on"] = static_cast<int>(MyData.MW10.laser_on);
	state["MW10"]["fan_on"] = static_cast<int>(MyData.MW10.fan_on);

	// ===== MW11 =====
	state["MW11"]["Z轴回原点指令"] = static_cast<int>(MyData.MW11.Z轴回原点指令);
	state["MW11"]["F轴回原点指令"] = static_cast<int>(MyData.MW11.F轴回原点指令);
	state["MW11"]["C轴回原点指令"] = static_cast<int>(MyData.MW11.C轴回原点指令);
	state["MW11"]["Z轴运动开始指令"] = static_cast<int>(MyData.MW11.Z轴运动开始指令);
	state["MW11"]["F轴运动开始指令"] = static_cast<int>(MyData.MW11.F轴运动开始指令);
	state["MW11"]["C轴运动开始指令"] = static_cast<int>(MyData.MW11.C轴运动开始指令);

	// ===== MW13_MW14 =====
	state["MW13_MW14"]["大充气阀状态"] = static_cast<int>(MyData.MW13_MW14.大充气阀状态);
	state["MW13_MW14"]["小充气阀状态"] = static_cast<int>(MyData.MW13_MW14.小充气阀状态);
	state["MW13_MW14"]["排气阀状态"] = static_cast<int>(MyData.MW13_MW14.排气阀状态);
	state["MW13_MW14"]["急停安全继电器状态"] = static_cast<int>(MyData.MW13_MW14.急停安全继电器状态);
	state["MW13_MW14"]["腔门插到位信号"] = static_cast<int>(MyData.MW13_MW14.腔门插到位信号);
	state["MW13_MW14"]["腔门锁上电状态"] = static_cast<int>(MyData.MW13_MW14.腔门锁上电状态);
	state["MW13_MW14"]["激光器有电信号"] = static_cast<int>(MyData.MW13_MW14.激光器有电信号);
	state["MW13_MW14"]["风机有电信号"] = static_cast<int>(MyData.MW13_MW14.风机有电信号);
	state["MW13_MW14"]["电机有电信号"] = static_cast<int>(MyData.MW13_MW14.电机有电信号);
	state["MW13_MW14"]["Z轴原点标志"] = static_cast<int>(MyData.MW13_MW14.Z轴原点标志);
	state["MW13_MW14"]["F轴原点标志"] = static_cast<int>(MyData.MW13_MW14.F轴原点标志);
	state["MW13_MW14"]["C轴原点标志"] = static_cast<int>(MyData.MW13_MW14.C轴原点标志);

	// ===== MW12 =====
	state["MW12"]["激光器报警"] = static_cast<int>(MyData.MW12.激光器报警);
	state["MW12"]["风机报警"] = static_cast<int>(MyData.MW12.风机报警);

	// ===== MW12_12_14 =====
	state["MW12_12_14"]["Z轴驱动器报警"] = static_cast<int>(MyData.MW12_12_14.Z轴驱动器报警);
	state["MW12_12_14"]["F轴驱动器报警"] = static_cast<int>(MyData.MW12_12_14.F轴驱动器报警);
	state["MW12_12_14"]["C轴驱动器报警"] = static_cast<int>(MyData.MW12_12_14.C轴驱动器报警);
	state["MW12_12_14"]["Z轴运动结束信号"] = static_cast<int>(MyData.MW12_12_14.Z轴运动结束信号);
	state["MW12_12_14"]["F轴运动结束信号"] = static_cast<int>(MyData.MW12_12_14.F轴运动结束信号);
	state["MW12_12_14"]["C轴运动结束信号"] = static_cast<int>(MyData.MW12_12_14.C轴运动结束信号);
	state["MW12_12_14"]["铺粉到位"] = static_cast<int>(MyData.MW12_12_14.铺粉到位);

	// ===== STOP =====
	state["STOP"]["Z轴运动停止信号"] = static_cast<int>(MyData.STOP.Z轴运动停止信号);
	state["STOP"]["F轴运动停止信号"] = static_cast<int>(MyData.STOP.F轴运动停止信号);
	state["STOP"]["C轴运动停止信号"] = static_cast<int>(MyData.STOP.C轴运动停止信号);

	// ===== 数值型 =====
	state["风压设定值"] = MyData.风压设定值;
	state["压力预警设定值"] = MyData.压力预警设定值;
	state["压力报警设定值"] = MyData.压力报警设定值;
	state["风压实际值"] = MyData.风压实际值;
	state["腔体压力"] = MyData.腔体压力;
	state["氧含量低精度"] = MyData.氧含量低精度;
	state["氧含量高精度"] = MyData.氧含量高精度;

	// ===== 运动参数 =====
	state["Z轴加速度"] = MyData.Z轴加速度;
	state["Z轴运动速度"] = MyData.Z轴运动速度;
	state["Z轴移动距离"] = MyData.Z轴移动距离;
	state["F轴加速度"] = MyData.F轴加速度;
	state["F轴运动速度"] = MyData.F轴运动速度;
	state["F轴移动距离"] = MyData.F轴移动距离;
	state["C轴加速度"] = MyData.C轴加速度;
	state["C轴运动速度"] = MyData.C轴运动速度;
	state["C轴移动距离"] = MyData.C轴移动距离;

	// ===== 当前位置 =====
	state["Z轴当前位置"] = MyData.Z轴当前位置;
	state["F轴当前位置"] = MyData.F轴当前位置;
	state["C轴当前位置"] = MyData.C轴当前位置;

	 return state;
} 


void ChuangRui_Control::Modbus() {//负责处理所有Modbus通信,核心
	Rs422 rs;//局部变量rs
	while (true) {//无限循环，确保函数持续运行不会退出
		{
			unique_lock<mutex>u2(modbus_mtx);//创建unique_lock对象u2，关联互斥锁modbus_mtx
			ModbusQueueCondVar.wait(u2, [&] { return !modbus_task.empty(); });//Lambda表达式作为等待条件,如果modbus_task队列为空，线程进入等待状态并释放锁
			rs = modbus_task.front(); //获取队列中的第一个（最旧的）任务元素,将任务数据复制到局部变量rs中,使用front()而不是pop()，只是读取不删除
			modbus_task.pop(); //从队列中移除第一个任务元素,这样队列中的任务就被"消费"掉了,必须先front()再pop()，不能直接pop()
		}//u2的析构函数自动调用unlock()释放modbus_mtx互斥锁

		switch (rs.mode) {//根据任务的模式（mode字段）进行分支选择,0（读）或1（写）

		case 0: {
			int ret = modbus_read_registers(rtu_var_, rs.address, rs.number, rs.dest);
			/*
			rtu_var_：全局Modbus RTU连接对象
			rs.address：起始寄存器地址（Modbus协议地址）
			rs.number：要读取的寄存器数量
			rs.dest：存储读取结果的缓冲区指针（uint16_t*类型）
			成功：返回读取的寄存器数量<—>失败：返回-1
			*/
			if (ret == -1) {
				this_thread::sleep_for(chrono::milliseconds(config_get(global_config, "ModbusRetryDelayMs", 100))); //如果失败，当前线程休眠,等待总线稳定或PLC准备好
				ret = modbus_read_registers(rtu_var_, rs.address, rs.number, rs.dest); //使用相同的参数重新调用Modbus读函数
				if (ret == -1)
					spdlog::error(
						"modbus read error: addr={}, num={}", rs.address, rs.number);
			}
			break;
		}
		case 1: {
			int ret = modbus_write_registers(rtu_var_, rs.address, rs.number, rs.dest);
			if (ret == -1) {
				this_thread::sleep_for(chrono::milliseconds(config_get(global_config, "ModbusRetryDelayMs", 100)));
				ret = modbus_write_registers(rtu_var_, rs.address, rs.number, rs.dest);
				if (ret == -1)
					spdlog::error(
						"modbus write error: addr={}, num={}", rs.address, rs.number);
			}
			break;
		}
			  //重试失败只输出错误，不崩溃
		}
		// end switch
	}
	// end while

}
// end Modbus


Result ChuangRui_Control::AxisMoveAnsyc(MotionControl::Axis axis, float distance) {
	return Result::Success;
}