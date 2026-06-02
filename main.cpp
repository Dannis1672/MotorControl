#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <map>
#include <functional>
#include "ChuangRui_Control.h"

using namespace std;

int main() {
    ChuangRui_Control ctrl;
    try {
    ctrl.ControlInitial();

   thread modbus_thread(Modbus);
   thread read_thread(ReadRegister);

    map<string, function<void(istringstream&)>> commands = 
    {//命令映射表
        {"AxisMove", [&ctrl](istringstream& iss) {
            cout << "正在执行Axis_Move" << endl;
            string ax; float d;
            iss >> ax >> d;
            ctrl.ChuangRui_Control::AxisMove(ax == "Z" ? Z : ax == "F" ? F : C, d);
            cout << "执行Axis_Move完成" << endl;
        }}

    
    };
	{//测试代码，一次只执行一条操作，输入exit退出测试    
    cout << "===== 测试开始 =====" << endl;
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
                cout << "未知命令: " << cmd << endl;
            }
        }
        }
    };


//示例：用户输入: "Z_F_move 20 20"   ----->   Z_F_move(20, 20)
    cout << "===== 测试完成 =====" << endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Unhandled unknown exception" << std::endl;
    }

    // Prevent console from closing immediately when the exe is started by double-click
    cout << "Press Enter to exit..." << endl;
    string _s;
    getline(cin, _s);

    return 0;

 }


    
