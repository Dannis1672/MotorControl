# MotoControl 使用手册

## 目录

- [概述](#概述)
- [CLI（命令行接口）使用方法](#cli命令行接口使用方法)
- [直接集成（C++ SDK 方式）](#直接集成c-sdk-方式)
- [配置说明](#配置说明)
- [构建指南](#构建指南)

---

## 概述

MotoControl 是一款基于 Modbus RTU 协议的增材制造设备运动控制软件，通过 `ChuangRui_Control` 类封装了对创瑞 PLC 的完整控制能力。支持以下功能：

- **三轴运动控制**：Z（成型缸）、F（供粉缸）、C（铺粉辊）的移动、回零、停止
- **数字量 I/O**：20 个按钮信号的写入（Light、ReSet、Door、GasCharge 等）
- **浮点参数设定**：10 个浮点参数（腔压、氧含量、风速、层数等）
- **系统状态监控**：完整的 Modbus 寄存器快照，JSON 格式
- **加工流程控制**：送粉检查（IsFeed）、加工开始/结束
- **异步架构**：Modbus 通信运行在独立线程，不阻塞命令交互

---

## CLI（命令行接口）使用方法

### 启动程序

```powershell
# Windows
.\moto_control.exe
```

程序启动后进入交互式 REPL 模式，等待输入命令。输入 `help` 查看帮助。

```
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
```

### 命令详解

#### 初始化与释放

```
init
```

连接 Modbus 设备并启动轮询线程，**所有其他命令依赖此命令先执行**。

```
free
```

释放资源、关闭连接。程序退出时也应调用此命令。

#### 轴运动控制

```
move <Z|F|C> <distance>
```

使指定轴运动指定距离。距离单位为配置中定义的工程单位。

```
move Z 10.5          # Z轴移动 10.5
move F 1.0           # F轴移动 1.0
move C 20.0          # C轴移动 20.0
```

```
tozero <Z|F|C>
```

使指定轴回零。

```
tozero C             # C轴回零
```

```
stop <Z|F|C>
```

紧急停止指定轴的运动。

```
stop Z               # 停止 Z 轴
```

#### 数字量输出（WriteBit）

```
set <Button> <0|1>
```

设置指定按钮的开关状态。0 = 关，1 = 开。

**有效按钮名称（共 20 个）：**

| 按钮名称 | 说明 |
|---|---|
| `Light` | 照明灯 |
| `ReSet` | 复位 |
| `Door` | 舱门 |
| `GasCharge` | 充气 |
| `LaserEnabled` | 激光使能 |
| `BackFlush` | 反吹 |
| `CharmberPressureTest` | 腔压测试 |
| `WholePressureTest` | 整体压力测试 |
| `Ventilate` | 排气 |
| `ProcessMode` | 加工模式 |
| `LaserCooler` | 激光冷却器 |
| `LaserPower` | 激光功率 |
| `LaserIndicate` | 激光指示 |
| `PowderFeed` | 粉末供给 |
| `Feed` | 送粉 |
| `CLeft` | C轴左移 |
| `CRight` | C轴右移 |
| `Fup` | F轴上升 |
| `FDown` | F轴下降 |
| `MotorPower` | 电机电源 |

示例：

```
set Light 1          # 开灯
set Door 0           # 关舱门
set Ventilate 1      # 开启排气
```

#### 浮点参数设定（WriteFloat）

```
float <Param> <value>
```

设置指定浮点型控制参数。

**有效参数名称（共 10 个）：**

| 参数名称 | 说明 |
|---|---|
| `CharmberPressure` | 腔体压力设定值 |
| `PressureDown` | 降压设定值 |
| `OxygenRatio` | 氧含量设定值 |
| `OxygenRatioDown` | 氧含量下降速率 |
| `WindSpeed` | 风速设定值 |
| `windPressure` | 风压设定值 |
| `Temperature` | 温度设定值 |
| `OverallLayers` | 总层数 |
| `CurrnetLayer` | 当前层数（名称按原始代码拼写） |
| `Test` | 测试参数 |

示例：

```
float OxygenRatio 0.25       # 设置氧含量为 25%
float Temperature 180.5      # 设置温度为 180.5
```

#### 系统状态查询

```
state
```

输出当前 PLC 全部寄存器值的 JSON 快照。包括：

- MW10：安全、灯、阀门、风机等 16 个位信号
- MW11：各轴回零/运动指令
- MW12~MW14：报警、限位、原点、到位信号
- 各轴的加速度/速度/当前位置/移动距离
- 风压/压力设定值、氧含量、腔压

示例输出片段：

```json
{
  "MW10": {
    "light": 0,
    "alarm": 0,
    "laser_enabled": 1
  },
  "Z轴当前位置": 2450,
  "C轴运动速度": 150.0
}
```

#### 送粉检查

```
feed <zd> <fup>
```

检查送粉是否完成。参数 `zd` 为 Z 轴位移量，`fup` 为 F 轴上升量。返回布尔值记录在日志中。

```
feed 0.5 1.0
```

#### 加工流程控制

```
begin
```

开始加工流程。

```
finish
```

结束加工流程。

### 退出程序

输入 `exit` 或 `quit` 退出 REPL，然后按 Enter 关闭程序。

---

## 直接集成（C++ SDK 方式）

如果需要在自己的应用程序中嵌入运动控制功能，直接使用 `MotionControl::Control` 抽象接口和 `ChuangRui_Control` 实现类。

### 依赖

- C++20 编译器
- CMake ≥ 3.20
- [spdlog](https://github.com/gabime/spdlog)（通过 FetchContent 自动下载）
- [nlohmann/json](https://github.com/nlohmann/json)（已包含在 `include/json.hpp`）
- [libmodbus](https://github.com/stephane/libmodbus)（已包含在 `include/libmodbus/`）

### 项目结构

```
MotorControl/
├── include/
│   ├── json.hpp              # nlohmann/json 单头文件
│   ├── modbus.h              # libmodbus 适配头
│   ├── moto_log.h            # spdlog 日志封装
│   └── libmodbus/            # libmodbus 源码
├── src/
│   ├── main.cpp              # CLI 入口（可作为集成参考）
│   └── control/
│       ├── MotionControl.h   # 抽象接口
│       ├── ChuangRui_Control.h
│       └── ChuangRui_Control.cpp
├── assets/config.json        # 配置文件
└── CMakeLists.txt
```

### 集成步骤

#### 1. 在 CMakeLists.txt 中链接库

```cmake
# 引入 moto_control 工程
add_subdirectory(path/to/MotorControl)

# 链接到你的可执行文件
target_link_libraries(your_app PRIVATE modbus spdlog::spdlog)
target_include_directories(your_app PRIVATE
    ${MotorControl_SOURCE_DIR}/include
    ${MotorControl_SOURCE_DIR}/src
)
```

或者直接复用 CMakeLists.txt 中已有的 `modbus` 静态库和 spdlog 依赖配置。

#### 2. 初始化日志和控制器

```cpp
#include <fstream>
#include "json.hpp"
#include "moto_log.h"
#include "control/ChuangRui_Control.h"

using json = nlohmann::json;

int main() {
    // 加载配置并初始化日志系统
    std::ifstream cfg_file("config.json");
    auto cfg = cfg_file.is_open() ? json::parse(cfg_file) : json{};
    moto::log::init(cfg);

    // 创建控制器实例
    ChuangRui_Control ctrl;
    ctrl.ControlInitial();  // 连接 Modbus，启动后台线程

    // ... 你的业务逻辑 ...

    ctrl.ControlFree();
    return 0;
}
```

#### 3. 调用运动控制接口

```cpp
// -- 轴运动 --
ctrl.AxisMove(MotionControl::Axis::Z, 10.5f);     // Z轴移动 10.5
ctrl.AxisToZero(MotionControl::Axis::F);           // F轴回零
ctrl.AxisStop(MotionControl::Axis::C);             // 停止 C 轴

// -- 数字量输出 --
ctrl.WriteBit(MotionControl::Light, true);         // 开灯
ctrl.WriteBit(MotionControl::Ventilate, false);    // 关排气

// -- 浮点参数 --
ctrl.WriteFloat(MotionControl::OxygenRatio, 0.25f);
ctrl.WriteFloat(MotionControl::Temperature, 180.5f);

// -- 系统状态 --
json state = ctrl.GetSystemState();
float zPos = state["Z轴当前位置"];
bool alarm = state["MW10"]["alarm"];

// -- 送粉检查 --
bool done = ctrl.IsFeed(0.5f, 1.0f);

// -- 加工流程 --
ctrl.ProcessBegin();
// ... 执行加工 ...
ctrl.ProcessFinish();
```

### 完整接口参考

`MotionControl::Control` 是抽象基类，定义了全部接口：

```cpp
class Control {
public:
    virtual ~Control() = default;

    // 生命周期
    virtual Result ControlInitial() = 0;
    virtual Result ControlFree() = 0;

    // I/O
    virtual Result WriteBit(UIButton io, bool value) = 0;
    virtual Result WriteFloat(UIFloat, float value) = 0;

    // 状态
    virtual nlohmann::json GetSystemState() = 0;

    // 轴控制
    virtual Result AxisMove(Axis axis, float distance) = 0;
    virtual Result AxisToZero(Axis axis) = 0;
    virtual Result AxisStop(Axis axis) = 0;

    // 加工流程
    virtual bool IsFeed(float zd, float fup) = 0;
    virtual void ProcessBegin() = 0;
    virtual void ProcessFinish() = 0;
};
```

返回值 `Result` 为 `MotionControl::Result::Success` 或 `Failure`。

### 枚举值速查

**Axis（轴）：**

| 枚举 | 说明 |
|---|---|
| `MotionControl::Axis::Z` | 成型缸 |
| `MotionControl::Axis::F` | 供粉缸 |
| `MotionControl::Axis::C` | 铺粉辊 |

**UIButton（数字量输出，20个）：**

| 枚举 | 说明 |
|---|---|
| `MotionControl::Light` | 照明灯 |
| `MotionControl::ReSet` | 复位 |
| `MotionControl::Door` | 舱门 |
| `MotionControl::GasCharge` | 充气 |
| `MotionControl::LaserEnabled` | 激光使能 |
| `MotionControl::BackFlush` | 反吹 |
| `MotionControl::CharmberPressureTest` | 腔压测试 |
| `MotionControl::WholePressureTest` | 整体压力测试 |
| `MotionControl::Ventilate` | 排气 |
| `MotionControl::ProcessMode` | 加工模式 |
| `MotionControl::LaserCooler` | 激光冷却器 |
| `MotionControl::LaserPower` | 激光功率 |
| `MotionControl::LaserIndicate` | 激光指示 |
| `MotionControl::PowderFeed` | 粉末供给 |
| `MotionControl::Feed` | 送粉 |
| `MotionControl::CLeft` | C轴左移 |
| `MotionControl::CRight` | C轴右移 |
| `MotionControl::Fup` | F轴上升 |
| `MotionControl::FDown` | F轴下降 |
| `MotionControl::MotorPower` | 电机电源 |

**UIFloat（浮点参数，10个）：**

| 枚举 | 说明 |
|---|---|
| `MotionControl::CharmberPressure` | 腔体压力设定值 |
| `MotionControl::PressureDown` | 降压设定值 |
| `MotionControl::OxygenRatio` | 氧含量设定值 |
| `MotionControl::OxygenRatioDown` | 氧含量下降速率 |
| `MotionControl::WindSpeed` | 风速设定值 |
| `MotionControl::windPressure` | 风压设定值 |
| `MotionControl::Temperature` | 温度设定值 |
| `MotionControl::OverallLayers` | 总层数 |
| `MotionControl::CurrnetLayer` | 当前层数 |
| `MotionControl::Test` | 测试参数 |

### 获取系统状态 JSON 字段说明

`GetSystemState()` 返回的 JSON 对象包含以下顶级字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `MW10.*` | 位域对象 | 安全、灯、阀门、风机、激光等 16 个状态位 |
| `MW11.*` | 位域对象 | 各轴回零/运动开始指令 |
| `MW12.*` | 位域对象 | 激光器/风机报警 |
| `MW12_12_14.*` | 位域对象 | 各轴驱动报警、限位、运动结束、铺粉到位 |
| `MW13_MW14.*` | 位域对象 | 阀门状态、急停、门锁、设备上电、原点标志 |
| `STOP.*` | 位域对象 | 各轴停止信号 |
| `Z/F/C轴加速度` | float | 对应轴的加速度设定值 |
| `Z/F/C轴运动速度` | float | 对应轴的速度设定值 |
| `Z/F/C轴移动距离` | float | 对应轴的移动距离设定值（命令值） |
| `Z/F/C轴当前位置` | int | 对应轴的当前位置（反馈值） |
| `风压设定值` | int | 风压设定值 |
| `压力预警设定值` | int | 压力预警阈值 |
| `压力报警设定值` | int | 压力报警阈值 |
| `氧含量低精度` | int | 氧含量低精度读数 |
| `氧含量高精度` | int | 氧含量高精度读数 |
| `风压实际值` | int | 风压实际值 |
| `腔体压力` | int | 腔体压力实际值 |

---

## 配置说明

配置文件 `config.json`（位于 `assets/` 目录，构建时复制到输出目录）控制以下方面：

### 日志配置

```json
{
  "logging": {
    "level": "debug",
    "async": true,
    "sinks": [
      { "type": "console", "enabled": true, "color": true },
      { "type": "file", "enabled": true, "path": "logs/moto_control.log", "max_size": 10485760, "max_files": 5 },
      { "type": "tcp", "enabled": false }
    ]
  }
}
```

### Modbus 通信配置

```json
{
  "rtu_set_": {
    "device": "COM2",
    "baud": 19200,
    "parity": "N",
    "data_bit": 8,
    "stop_bit": 1
  },
  "slave_id": 1,
  "byte_timeout_us": 100000,
  "response_timeout_us": 100000
}
```

- `device`：串口号，如 `COM2`
- `baud`：波特率，默认 19200
- `parity`：校验位，`N`（无）、`E`（偶）、`O`（奇）
- `data_bit`：数据位（通常 8）
- `stop_bit`：停止位（通常 1）
- `slave_id`：Modbus 从站地址

### 轴运动参数

```json
{
  "Z_Move_Distance": 0.5,
  "Z_Accelerate": 12.0,
  "Z_Velocity": 3.0,

  "F_Move_Distance": 1.0,
  "F_Accelerate": 12.0,
  "F_Velocity": 3.0,

  "C_Move_Distance": 20.0,
  "C_Accelerate": 350.0,
  "C_Velocity": 150.0
}
```

### 电子齿轮比

```json
{
  "Motor_ratio_Z": 5000.0,
  "Motor_ratio_F": 5000.0,
  "Motor_ratio_C": 95.2
}
```

用于将工程单位（mm）转换为电机脉冲数。

### 其他参数

| 字段 | 说明 |
|---|---|
| `ModbusRetryDelayMs` | Modbus 重试间隔（毫秒） |
| `ReadInterval` | 寄存器轮询间隔（毫秒） |


---

## 构建指南

### 前置条件

- Windows 10/11（目前仅支持 Windows）
- Visual Studio 2022（含 MSVC C++20 工具链）
- CMake ≥ 3.20
- Git

### 构建步骤

```powershell
# 克隆项目
git clone <repository-url>
cd MotorControl

# 配置（首次需要下载 spdlog）
cmake -B build -G "Visual Studio 17 2022" -A x64

# 编译 Debug 版本
cmake --build build --config Debug

# 编译 Release 版本
cmake --build build --config Release
```

产物位置：
- `build/Debug/moto_control.exe`
- `build/Release/moto_control.exe`

配置文件 `config.json` 会自动复制到输出目录。运行前确认串口号 `COM2` 等参数与实际硬件匹配。

### 打包

```powershell
# 生成 ZIP 包
cmake --build build --config Release --target PACKAGE
```

---

## 注意事项

1. **安全操作**：运动控制命令直接操作物理设备，请务必确认设备状态后再发送命令。
2. **Modbus 连接**：`init` 命令只有在正确的 COM 口和参数配置下才能成功连接 PLC。无硬件时 Modbus 读会失败但程序不会崩溃。
3. **线程模型**：`ChuangRui_Control` 内部维护独立的 Modbus 通信线程和寄存器轮询线程，`ControlInitial` 启动这些线程，`ControlFree` 停止。
4. **日志**：所有操作通过 spdlog 记录到控制台和文件（`logs/moto_control.log`），调试时建议开启 `debug` 级别。
