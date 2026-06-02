
#include"Rtc5Scan.h"

const UINT      ResetCompletely(UINT_MAX);

const UINT      HalfPeriod = global_config.value("HalfPeriod", 640U);  //半周期
const UINT      PulseLength = global_config.value("PulseLength", 1275U);  //脉冲长度

const long int  LaserOnDelay = global_config.value("LaserOnDelay", 1400L);  //开光延迟
const UINT      LaserOffDelay = global_config.value("LaserOffDelay", 1600U);  //关光延迟

const double    MarkSpeed = global_config.value("MarkSpeed", 5882);           //  1000mm/s
const double    JumpSpeed = global_config.value("JumpSpeed", 17646);          //  3000mm/s



int scanInitial()//初始化RTC5控制卡，配置激光和振镜基本参数，无参数版本
{
	//Scan Calibration factor K [Bit/mm]
	//初始化 DLL 和获取卡信息
	UINT ErrorCode = init_rtc5_dll();// 初始化RTC5 DLL
	UINT rtcCards = rtc5_count_cards();//获取RTC5卡数量，也对应最后一张卡的编号
	ErrorCode = select_rtc(rtcCards);// 选择最后一张卡
	stop_execution();

	//加载加载校正和程序文件，分配内存
	ErrorCode = load_correction_file("c:\\home\\YMBuild\\RTC5_pbam\\test_sl3.ct5",  1U, 2U);
	// 加载校正文件，表格编号 = 1，模式 = 2（3D矫正模式）
	ErrorCode = load_program_file("c:\\home\\YMBuild\\RTC5_pbam");// 加载程序文件
	select_cor_table(1, 0);//选择校正表格，表格A = 1，表格B = 0
	reset_error(-1);  //重置所有错误状态
	config_list(UINT_MAX, 0U);//配置列表，内存1 = UINT_MAX（使用所有可用内存），内存2 = 0（不使用第二块内存）

	//设置激光参数 1bit=1/64us
	set_laser_mode(1);	    //激光模式1，CO2激光器
	set_firstpulse_killer(20 * 64U);	// 首脉冲抑制，防止第一个脉冲能量过高，1bit = 1/64us
	set_laser_control(0);//0x18 bit#4 and #5 low active;0 high active
	set_standby(640U, 64U);      //1bit=1/64us, 激光半周期，640/64=10us，脉冲宽度，64/64 = 1微秒

	//设置振镜延迟
	set_start_list(1);//开始构建列表 1，后续命令写入列表而不是立即执行。
	set_scanner_delays(20U, 15U, 5U);// 扫描器延迟，1bit = 10微秒
	set_laser_delays(370L, 480U);// 激光延迟，1bit = 0.5微秒，RTC4 1bit = 1微秒
	set_laser_pulses(640U, 1275U);//1bit=1/64us, 激光半周期，640/64=10us，脉冲宽度，1275/64 = 19.92微秒	
	set_jump_speed(JumpSpeed);
	set_mark_speed(MarkSpeed);//1 bit per ms
	set_end_of_list();
	execute_list(1);

	set_delay_mode(1, 0, 5, 0, 588);//模式, 预留, 变向延迟, 预留, 跳转延迟
	return 0;
}

int scanInitial(const std::string& rtc_crt_file)//初始化RTC5控制卡，配置激光和振镜基本参数，有参数版本
{
	//Scan Calibration factor K [Bit/mm]
	//初始化 DLL 和获取卡信息
	UINT ErrorCode = init_rtc5_dll();// 初始化RTC5 DLL
	UINT rtcCards = rtc5_count_cards();//获取RTC5卡数量，也对应最后一张卡的编号
	ErrorCode = select_rtc(rtcCards);// 选择最后一张卡
	stop_execution();

	//加载加载校正和程序文件，分配内存
	ErrorCode = load_correction_file(rtc_crt_file.c_str(), 1U, 2U);
	// 加载校正文件，表格编号 = 1，模式 = 2（3D矫正模式）
	ErrorCode = load_program_file("c:\\home\\YMBuild\\RTC5_pbam");// 加载程序文件
	select_cor_table(1, 0);//选择校正表格，表格A = 1，表格B = 0
	reset_error(-1);  //重置所有错误状态
	config_list(UINT_MAX, 0U);//配置列表，内存1 = UINT_MAX（使用所有可用内存），内存2 = 0（不使用第二块内存）

	//设置激光参数 1bit=1/64us
	set_laser_mode(1);	    //激光模式1，CO2激光器
	set_firstpulse_killer(20 * 64U);	// 首脉冲抑制，防止第一个脉冲能量过高，1bit = 1/64us
	set_laser_control(0);//0x18 bit#4 and #5 low active;0 high active
	set_standby(640U, 64U);      //1bit=1/64us, 激光半周期，640/64=10us，脉冲宽度，64/64 = 1微秒

	//设置振镜延迟
	set_start_list(1);//开始构建列表 1，后续命令写入列表而不是立即执行。
	set_scanner_delays(20U, 15U, 5U);// 扫描器延迟，1bit = 10微秒
	set_laser_delays(LaserOnDelay, LaserOffDelay);// 激光延迟，1bit = 0.5微秒，RTC4 1bit = 1微秒
	set_laser_pulses(HalfPeriod, PulseLength);//1bit=1/64us, 激光半周期，640/64=10us，脉冲宽度，1275/64 = 19.92微秒	
	set_jump_speed(JumpSpeed);
	set_mark_speed(MarkSpeed);//1 bit per ms
	set_end_of_list();
	execute_list(1);

	return 0;
}