#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>
#include <map>
#include <functional>
#include "moto_log.h"
#include "control/ChuangRui_Control.h"
#include <Windows.h>
using namespace std;

int main() {
    // 设置控制台输出为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
    // 设置控制台输入为 UTF-8（可选）
    SetConsoleCP(CP_UTF8);


    // 加载配置文件，初始化日志系统
    {
        std::ifstream log_cfg_file("config.json");
        if (log_cfg_file.is_open()) {
            auto cfg = nlohmann::json::parse(log_cfg_file);
            moto::log::init(cfg);
        } else {
            // config.json 不存在时使用全默认值（仅控制台输出）
            moto::log::init(nlohmann::json{});
        }
    }

    ChuangRui_Control ctrl;

    try {

    ctrl.ControlInitial();

    map<string, function<void(istringstream&)>> commands =
    {//命令映射表
        {"AxisMove", [&ctrl](istringstream& iss) {
            spdlog::info("正在执行Axis_Move");
            string ax; float d;
            iss >> ax >> d;

            ctrl.AxisMoveAnsyc(ax == "Z" ? MotionControl::Z : ax == "F" ? MotionControl::F : MotionControl::C, d);
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
