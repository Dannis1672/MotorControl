#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>
#include <map>
#include <functional>
#include <vector>
#include <iomanip>
#include <cmath>

#include <spdlog/spdlog.h>

#include "moto_log.h"
#ifdef _WIN32
#include <Windows.h>
#endif
#include "control/ChuangRui_Control.h"

using json = nlohmann::json;

// ============================================================
// 枚举值与字符串之间的映射
// ============================================================

struct AxisNames {
    static MotionControl::Axis parse(const std::string& s) {
        if (s == "Z" || s == "z") return MotionControl::Axis::Z;
        if (s == "F" || s == "f") return MotionControl::Axis::F;
        if (s == "C" || s == "c") return MotionControl::Axis::C;
        throw std::runtime_error("未知轴名: " + s + " (有效值: Z, F, C)");
    }
    static const char* name(MotionControl::Axis a) {
        switch (a) {
            case MotionControl::Axis::Z: return "Z";
            case MotionControl::Axis::F: return "F";
            case MotionControl::Axis::C: return "C";
        }
        return "?";
    }
};

struct UIButtonNames {
    static MotionControl::UIButton parse(const std::string& s) {
        static const std::map<std::string, MotionControl::UIButton> tbl = {
            {"Light",                 MotionControl::Light},
            {"ReSet",                 MotionControl::ReSet},
            {"Door",                  MotionControl::Door},
            {"GasCharge",             MotionControl::GasCharge},
            {"LaserEnabled",          MotionControl::LaserEnabled},
            {"BackFlush",             MotionControl::BackFlush},
            {"CharmberPressureTest",  MotionControl::CharmberPressureTest},
            {"WholePressureTest",     MotionControl::WholePressureTest},
            {"Ventilate",             MotionControl::Ventilate},
            {"ProcessMode",           MotionControl::ProcessMode},
            {"LaserCooler",           MotionControl::LaserCooler},
            {"LaserPower",            MotionControl::LaserPower},
            {"LaserIndicate",         MotionControl::LaserIndicate},
            {"PowderFeed",            MotionControl::PowderFeed},
            {"Feed",                  MotionControl::Feed},
            {"CLeft",                 MotionControl::CLeft},
            {"CRight",                MotionControl::CRight},
            {"Fup",                   MotionControl::Fup},
            {"FDown",                 MotionControl::FDown},
            {"MotorPower",            MotionControl::MotorPower},
        };
        auto it = tbl.find(s);
        if (it != tbl.end()) return it->second;
        throw std::runtime_error("未知按钮: " + s);
    }
    static void list() {
        std::cout << "有效按钮名称:\n";
        const char* names[] = {
            "Light", "ReSet", "Door", "GasCharge", "LaserEnabled",
            "BackFlush", "CharmberPressureTest", "WholePressureTest", "Ventilate",
            "ProcessMode", "LaserCooler", "LaserPower", "LaserIndicate",
            "PowderFeed", "Feed", "CLeft", "CRight", "Fup", "FDown", "MotorPower"
        };
        for (const auto& n : names) std::cout << "  " << n << "\n";
    }
};

struct UIFloatNames {
    static MotionControl::UIFloat parse(const std::string& s) {
        static const std::map<std::string, MotionControl::UIFloat> tbl = {
            {"CharmberPressure",    MotionControl::CharmberPressure},
            {"PressureDown",        MotionControl::PressureDown},
            {"OxygenRatio",         MotionControl::OxygenRatio},
            {"OxygenRatioDown",     MotionControl::OxygenRatioDown},
            {"WindSpeed",           MotionControl::WindSpeed},
            {"windPressure",        MotionControl::windPressure},
            {"Temperature",         MotionControl::Temperature},
            {"OverallLayers",       MotionControl::OverallLayers},
            {"CurrnetLayer",        MotionControl::CurrnetLayer},
            {"Test",                MotionControl::Test},
        };
        auto it = tbl.find(s);
        if (it != tbl.end()) return it->second;
        throw std::runtime_error("未知浮点参数: " + s);
    }
    static void list() {
        std::cout << "有效浮点参数名称:\n";
        const char* names[] = {
            "CharmberPressure", "PressureDown", "OxygenRatio",
            "OxygenRatioDown", "WindSpeed", "windPressure", "Temperature",
            "OverallLayers", "CurrnetLayer", "Test"
        };
        for (const auto& n : names) std::cout << "  " << n << "\n";
    }
};


// ============================================================
// 命令行接口类
// ============================================================

class CommandLine {
public:
    explicit CommandLine(ChuangRui_Control& ctrl) : ctrl_(ctrl) {}

    void run();

private:
    ChuangRui_Control& ctrl_;

    // ---- 命令处理函数 ----
    void cmd_init(const std::vector<std::string>& args);
    void cmd_free(const std::vector<std::string>& args);
    void cmd_axis_move(const std::vector<std::string>& args);
    void cmd_axis_tozero(const std::vector<std::string>& args);
    void cmd_axis_stop(const std::vector<std::string>& args);
    void cmd_write_bit(const std::vector<std::string>& args);
    void cmd_write_float(const std::vector<std::string>& args);
    void cmd_state(const std::vector<std::string>& args);
    void cmd_feed(const std::vector<std::string>& args);
    void cmd_process_begin(const std::vector<std::string>& args);
    void cmd_process_finish(const std::vector<std::string>& args);
    void cmd_help(const std::vector<std::string>& args);

    using CommandFunc = std::function<void(const std::vector<std::string>&)>;
    std::map<std::string, CommandFunc> commands_;

    void print_help();
    void print_result(MotionControl::Result r, const std::string& op);

    // 参数解析辅助
    static void require_args(const std::vector<std::string>& args,
                             size_t count, const std::string& usage);
};

// ------------------------------------------------------------
// 命令注册与主循环
// ------------------------------------------------------------

void CommandLine::run() {
    commands_ = {
        {"init",            [this](auto& a) { cmd_init(a); }},
        {"free",            [this](auto& a) { cmd_free(a); }},
        {"move",            [this](auto& a) { cmd_axis_move(a); }},
        {"tozero",          [this](auto& a) { cmd_axis_tozero(a); }},
        {"stop",            [this](auto& a) { cmd_axis_stop(a); }},
        {"set",             [this](auto& a) { cmd_write_bit(a); }},
        {"float",           [this](auto& a) { cmd_write_float(a); }},
        {"state",           [this](auto& a) { cmd_state(a); }},
        {"feed",            [this](auto& a) { cmd_feed(a); }},
        {"begin",           [this](auto& a) { cmd_process_begin(a); }},
        {"finish",          [this](auto& a) { cmd_process_finish(a); }},
        {"help",            [this](auto& a) { cmd_help(a); }},
    };

    spdlog::info("===== 命令行接口启动 =====");

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::vector<std::string> args;
        std::string tok;
        while (iss >> tok) args.push_back(tok);

        const std::string cmd = args[0];
        if (cmd == "exit" || cmd == "quit") break;

        auto it = commands_.find(cmd);
        if (it != commands_.end()) {
            try {
                it->second(args);
            } catch (const std::exception& e) {
                spdlog::error("命令执行失败: {}", e.what());
            }
        } else {
            spdlog::warn("未知命令: {}。输入 'help' 查看帮助。", cmd);
        }
    }

    spdlog::info("===== 命令行接口结束 =====");
}

// ------------------------------------------------------------
// 命令实现
// ------------------------------------------------------------

void CommandLine::print_result(MotionControl::Result r, const std::string& op) {
    if (r == MotionControl::Result::Success) {
        spdlog::info("{}: 成功", op);
    } else {
        spdlog::error("{}: 失败", op);
    }
}

void CommandLine::require_args(const std::vector<std::string>& args,
                               size_t count, const std::string& usage) {
    if (args.size() < count) {
        throw std::runtime_error("参数不足。用法: " + usage);
    }
}

// --- init ---
void CommandLine::cmd_init(const std::vector<std::string>&) {
    print_result(ctrl_.ControlInitial(), "ControlInitial");
}

// --- free ---
void CommandLine::cmd_free(const std::vector<std::string>&) {
    print_result(ctrl_.ControlFree(), "ControlFree");
}

// --- move <axis> <distance> ---
void CommandLine::cmd_axis_move(const std::vector<std::string>& args) {
    require_args(args, 3, "move <Z|F|C> <distance>");
    auto axis = AxisNames::parse(args[1]);
    float dist = std::stof(args[2]);
    spdlog::info("AxisMove {} {}", AxisNames::name(axis), dist);
    print_result(ctrl_.AxisMove(axis, dist), "AxisMove");
}

// --- tozero <axis> ---
void CommandLine::cmd_axis_tozero(const std::vector<std::string>& args) {
    require_args(args, 2, "tozero <Z|F|C>");
    auto axis = AxisNames::parse(args[1]);
    spdlog::info("AxisToZero {}", AxisNames::name(axis));
    print_result(ctrl_.AxisToZero(axis), "AxisToZero");
}

// --- stop <axis> ---
void CommandLine::cmd_axis_stop(const std::vector<std::string>& args) {
    require_args(args, 2, "stop <Z|F|C>");
    auto axis = AxisNames::parse(args[1]);
    spdlog::info("AxisStop {}", AxisNames::name(axis));
    print_result(ctrl_.AxisStop(axis), "AxisStop");
}

// --- set <button> <0|1> ---
void CommandLine::cmd_write_bit(const std::vector<std::string>& args) {
    require_args(args, 3, "set <Button> <0|1>");
    auto btn = UIButtonNames::parse(args[1]);
    bool val = (std::stoi(args[2]) != 0);
    spdlog::info("WriteBit {} = {}", args[1], val);
    print_result(ctrl_.WriteBit(btn, val), "WriteBit");
}

// --- float <param> <value> ---
void CommandLine::cmd_write_float(const std::vector<std::string>& args) {
    require_args(args, 3, "float <Param> <value>");
    auto p = UIFloatNames::parse(args[1]);
    float val = std::stof(args[2]);
    spdlog::info("WriteFloat {} = {}", args[1], val);
    print_result(ctrl_.WriteFloat(p, val), "WriteFloat");
}

// --- state ---
void CommandLine::cmd_state(const std::vector<std::string>&) {
    json state = ctrl_.GetSystemState();
    std::cout << state.dump(2) << std::endl;
}

// --- feed <zd> <fup> ---
void CommandLine::cmd_feed(const std::vector<std::string>& args) {
    require_args(args, 3, "feed <zd> <fup>");
    float zd  = std::stof(args[1]);
    float fup = std::stof(args[2]);
    spdlog::info("IsFeed(zd={}, fup={}) = {}", zd, fup, ctrl_.IsFeed(zd, fup));
}

// --- begin ---
void CommandLine::cmd_process_begin(const std::vector<std::string>&) {
    spdlog::info("ProcessBegin");
    ctrl_.ProcessBegin();
}

// --- finish ---
void CommandLine::cmd_process_finish(const std::vector<std::string>&) {
    spdlog::info("ProcessFinish");
    ctrl_.ProcessFinish();
}

// --- help ---
void CommandLine::cmd_help(const std::vector<std::string>&) {
    print_help();
}

void CommandLine::print_help() {
    std::cout << R"(
================================================================
  MotoControl CLI — 运动控制命令行接口
================================================================

命令列表:
  init                      初始化控制资源 (ControlInitial)
  free                      释放控制资源 (ControlFree)

  move <Z|F|C> <dist>       轴移动 (AxisMove)
  tozero <Z|F|C>            轴回零 (AxisToZero)
  stop <Z|F|C>              轴停止 (AxisStop)

  set <Button> <0|1>        写数字量输出 (WriteBit)
  float <Param> <value>     写浮点参数 (WriteFloat)

  state                     获取系统状态 JSON (GetSystemState)
  feed <zd> <fup>           送粉完成检查 (IsFeed)
  begin                     开始加工 (ProcessBegin)
  finish                    结束加工 (ProcessFinish)

  help                      显示本帮助
  exit / quit               退出程序

----------------------------------------------------------------
按钮名称 (用于 set 命令):
)";
    UIButtonNames::list();
    std::cout << R"(
浮点参数名称 (用于 float 命令):
)";
    UIFloatNames::list();
    std::cout << R"(
示例:
  init
  move Z 10.5
  tozero C
  stop F
  set Light 1
  float OxygenRatio 0.25
  state
  feed 0.5 1.0
  begin
  finish
  exit

===============================================================
)" << std::endl;
}


// ============================================================
// main
// ============================================================

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    // 加载配置并初始化日志系统
    {
        std::ifstream log_cfg_file("config.json");
        if (log_cfg_file.is_open()) {
            auto cfg = json::parse(log_cfg_file);
            moto::log::init(cfg);
        } else {
            moto::log::init(json{});
        }
    }

    ChuangRui_Control ctrl;

    try {
        CommandLine cli(ctrl);
        cli.run();
    } catch (const std::exception& e) {
        spdlog::critical("未捕获异常: {}", e.what());
    } catch (...) {
        spdlog::critical("未捕获的未知异常");
    }

    std::cout << "按 Enter 退出..." << std::endl;
    std::string _s;
    std::getline(std::cin, _s);
    return 0;
}