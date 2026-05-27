#include<chrono>
#include<mutex>
#include<shared_mutex>
#include<thread>
#include<condition_variable>
#include<fstream>
#include<iostream>
#include"json.hpp"
#include"control.h"

using namespace std;
using json = nlohmann::json;


//void Manufacturing(CLayers clayers, float &interval, double &rotate_angle,
//	float& Z_Delta);//功率和扫描速度在ScanInitial

mutex rs422_mtx;
mutex task_mtx;
mutex modbus_mtx;
shared_mutex mutex_;
condition_variable queueCondVar;
condition_variable ModbusQueueCondVar;
std::queue<Rs422>modbus_task;

json global_config;
float Motor_ratio[3] = { 5000,5000,95.2 };
void Config_innitial() //读取json配置文件并赋值给全局变量global_config
{
	std::ifstream file("config.json");
	global_config = json::parse(file);
	file.close();
}

template<typename T>
void Config_set(const std::string& group, const std::string& key, const T& value)
{
	global_config[group][key] = value;//生产过程更新global_config
}
template<typename T>
void Config_set(const std::string& key, const T& value)
{
	global_config[key] = value;//重载函数，生产过程更新global_config
}

void ModbusInitial()//初始化函数
{
	//初始化rtu_set_结构体
	global_config["rtu_set_"]["device"].get<std::string>().copy(rtu_set_.device, sizeof(rtu_set_.device) - 1);//串行端口名称
	rtu_set_.device[sizeof(rtu_set_.device) - 1] = '\0';//末尾置0

	rtu_set_.baud = global_config["rtu_set_"]["baud"].get<int>();//波特率
	rtu_set_.parity = global_config["rtu_set_"]["parity"].get<std::string>().at(0);
	rtu_set_.data_bit = global_config["rtu_set_"]["data_bit"].get<int>();//指定数据的位数
	rtu_set_.stop_bit = global_config["rtu_set_"]["stop_bit"].get<int>();//指定停止位位数

	//modbus连接,创建一个 Modbus RTU 上下文（对象）,rtu_var_ 是一个 modbus_t* 类型的指针，后续所有操作都通过它来进行。
	rtu_var_ = modbus_new_rtu(rtu_set_.device, rtu_set_.baud, rtu_set_.parity, rtu_set_.data_bit, rtu_set_.stop_bit);

	//设置从站
	int r = modbus_set_slave(rtu_var_, 1);
	if (r == -1) { cout << "Modbus_set_slave error" << endl; }

	//打开串口，建立物理连接
	r = modbus_connect(rtu_var_);
	if (r == -1) { cout << "Modbus_connect error" << endl; }

	//主站发送完请求后，等待从站回复的最长时间 100 毫秒
	modbus_set_byte_timeout(rtu_var_, 0, 100000);
	modbus_set_response_timeout(rtu_var_, 0, 100000);
	//这行是在准备要写入的数据
	unsigned int initial = 0;
	r = modbus_write_registers(rtu_var_, 10, 2, (uint16_t*)&initial);//用指针强转16位
	if (r == -1) { cout << "Initial error" << endl;; }

//根据配置文件初始化Z_F_C方向运动参数，单位mm和s

    Z_Move_Distance = global_config["Z_Move_Distance"].get<float>();
	Z_Accelerate = global_config["Z_Accelerate"].get<float>();
	Z_Velocity = global_config["Z_Velocity"].get<float>();

	F_Move_Distance = global_config["F_Move_Distance"].get<float>();
	F_Accelerate = global_config["F_Accelerate"].get<float>();
	F_Velocity = global_config["F_Velocity"].get<float>();

	C_Move_Distance = global_config["C_Move_Distance"].get<float>();
	C_Accelerate = global_config["C_Accelerate"].get<float>();
	C_Velocity = global_config["C_Velocity"].get<float>();

	for (int i = 0; i < 3; ++i) {//初始化Motor_ratios数组
	Motor_ratio[i] = global_config["Motor_ratio"][i].get<float>();
	}

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


}
void ModbusTaskPush(Rs422& rs422) {//把一个 Modbus 通信任务放入队列，并通知消费者线程来处理
	{
		lock_guard<mutex>lg2(modbus_mtx);
		modbus_task.push(rs422);
	}
	ModbusQueueCondVar.notify_one();
}
void taskPush(FunCode& funcode) {//和上面完全一样，只是队列名和数据类型不同。放入的是 FunCode（功能码）
	{
		lock_guard<std::mutex> lg1(task_mtx);
		task.push(funcode);
	}
	queueCondVar.notify_one();
}
CharmRayPlcRegs& get() {//读数据
	shared_lock<shared_mutex>lck(mutex_);
	return MyData;
}
CharmRayPlcRegs* write() {//写数据
	unique_lock<shared_mutex>lck(mutex_);
	return &MyData;
}

void ReadRegister() {
	Rs422 rs422;//rs422生命周期
	while (true) {//无限循环，持续读取PLC数据
		int read_interval = global_config.value("ReadInterval", 200);
		this_thread::sleep_for(chrono::milliseconds(read_interval));//当前线程休眠200毫秒，控制读取频率为5Hz

		rs422.change(0, 10, 56, (uint16_t*)write());
		//这里的write()锁的生命周期
		/*参数1 0：读取模式；参数2 10：起始地址（Modbus地址40011，即MW10）；
		参数3 56：读取56个寄存器；参数4 (uint16_t*)write()：获取可写指针作为读取缓冲区
		读取的数据会写入MyData结构体*/

		ModbusTaskPush(rs422);//将配置好的读取任务推送到Modbus任务队列
	}
}

void Z_F_move(float Z_distance, float F_distance) {//Z轴和F轴联动移动函数，Z轴目标距离，F轴目标距离
	out_data.Z轴移动距离 = Motor_ratio[Z] * Z_distance;
	out_data.F轴移动距离 = Motor_ratio[F] * F_distance;
	Rs422 rs422;
	rs422.change(1, 24, 8, (uint16_t*)&out_data.Z轴移动距离);
	ModbusTaskPush(rs422);

	out_data.MW11.Z轴运动开始指令 = 1;
	out_data.MW11.F轴运动开始指令 = 1;
	rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
	ModbusTaskPush(rs422);


	int count = 0;
	while (count < 100) {
		count++;
		if (count == 1)
			this_thread::sleep_for(chrono::milliseconds(500));
		else
			this_thread::sleep_for(chrono::milliseconds(10));
		if (get().MW12_12_14.F轴运动结束信号 == 1) {
			out_data.MW11.Z轴运动开始指令 = 0;
			out_data.MW11.F轴运动开始指令 = 0;
			rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
			ModbusTaskPush(rs422);
			break;
		}
	}
	if (count == 99) cout << "未到位" << endl;

}

void Axis_Move(Axis axis, float distance) {
	//用于传参
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

	int count = 0;

	int velocity = 1;  //当前运动轴速度
	switch (axis) {
	case Z: velocity = Z_Velocity; break;
	case F: velocity = F_Velocity; break;
	case C: velocity = C_Velocity; break;
	}
	int timeout = (abs(distance)/velocity + 2)*100;//单位为ms
	timeout = max(timeout,600);

	while (count < timeout) {
		count++;	//第一次sleep时间大一些

		if (count == 1)
			this_thread::sleep_for(chrono::milliseconds(500));
		else
			this_thread::sleep_for(chrono::milliseconds(10));//读到旧值很可怕
		bool moveStopFlag = (*(uint16_t*)(&get().MW12_12_14) & (1 << (12 + axis))) >> (12 + axis);
		//if (((*(uint16_t*)( &get().MW12_12_14)  &  (1 << (axis + 12)) )>> 12)== 1) {//这句话you'wen'ti
		if (moveStopFlag == 1) {
			//回零
			if (abs(distance) < 1e-6) {
				*(uint16_t*)&out_data.MW11 &= (~(1 << axis));
			}
			//开始运动
			else
			{
				*(uint16_t*)&out_data.MW11 &= (~(1 << (axis + 3)));//3为bit值偏移量
			}
			rs422.change(1, 11, 1, (uint16_t*)&out_data.MW11);
			ModbusTaskPush(rs422);
			break;
		}
	}
	if (count == timeout) cout << "未到位" << endl;



}

void Modbus() {//负责处理所有Modbus通信,核心
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
				this_thread::sleep_for(chrono::milliseconds(100)); //如果失败，当前线程休眠100毫秒,等待总线稳定或PLC准备好
				ret = modbus_read_registers(rtu_var_, rs.address, rs.number, rs.dest); //使用相同的参数重新调用Modbus读函数
				if (ret == -1)cout << "modbus read error" << endl;//输出错误信息到控制台"modbus read error"
			}
			break;
		}
		case 1: {
			int ret = modbus_write_registers(rtu_var_, rs.address, rs.number, rs.dest);
			if (ret == -1) {
				this_thread::sleep_for(chrono::milliseconds(100));
				ret = modbus_write_registers(rtu_var_, rs.address, rs.number, rs.dest);
				if (ret == -1)
					cout << "modbus write error" << endl;
			}
			break;
		}
			  //重试失败只输出错误，不崩溃
		}

	}


}