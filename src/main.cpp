#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <map>
#include <functional>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "control/ChuangRui_Control.h"
#include <Windows.h>
using namespace std;

int main() {
    // 设置控制台输出为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
    // 设置控制台输入为 UTF-8（可选）
    SetConsoleCP(CP_UTF8);


    auto console = spdlog::stdout_color_mt("console");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::debug);

    ChuangRui_Control ctrl;
    try {
    ctrl.ControlInitial();

   thread modbus_thread(Modbus);
   thread read_thread(ReadRegister);

    map<string, function<void(istringstream&)>> commands =
    {//命令映射表
        {"AxisMove", [&ctrl](istringstream& iss) {
            spdlog::info("正在执行Axis_Move");
            string ax; float d;
            iss >> ax >> d;
            ctrl.AxisMove(ax == "Z" ? Z : ax == "F" ? F : C, d);
            spdlog::info("执行Axis_Move完成");
        }}


    };
	{//测试代码，一次只执行一条操作，输入exit退出测试
    spdlog::info("===== 测试开始 =====");
    string line;
    while (getline(cin, line))
    {





        if (line == "exit") break;

        else
        {
            istringstream iss(line);
            string cmd;
            iss >> cmd;
            auto it = commands.find(cmd);
            if (it != commands.end()) {
                it->second(iss);
            } else {
                spdlog::warn("未知命令: {}", cmd);
            }
        }
        }
    };


    spdlog::info("===== 测试完成 =====");
    }
    catch (const std::exception& e) {
        spdlog::critical("Unhandled exception: {}", e.what());
    }
    catch (...) {
        spdlog::critical("Unhandled unknown exception");
    }

    cout << "Press Enter to exit..." << endl;
    string _s;
    getline(cin, _s);

    return 0;

 }
