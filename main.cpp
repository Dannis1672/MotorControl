#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <map>
#include <functional>
#include "control.h"
using namespace std;

int main() {

    Config_innitial();
    ModbusInitial();

   // thread modbus_thread(Modbus);
   //thread read_thread(ReadRegister);

    map<string, function<void(istringstream&)>> commands = 
    {//命令映射表
        {"Axis_Move", [](istringstream& iss) {
            cout << "正在执行Axis_Move" << endl;
            string ax; float d;
            iss >> ax >> d;
            Axis_Move(ax == "Z" ? Z : ax == "F" ? F : C, d);
            cout << "执行Axis_Move完成" << endl;
        }},

        {"Z_F_move", [](istringstream& iss) {
            cout << "正在执行Z_F_move" << endl;
            float z, f;
            iss >> z >> f;
            Z_F_move(z, f);
            cout << "执行Z_F_move完成" << endl;
        }}


    };



    {//测试代码，一次只执行一条移动函数
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


    return 0;

 }


    
