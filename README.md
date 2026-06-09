# moto_control

基于 Modbus RTU 的增材制造设备运动控制软件，封装创瑞 PLC 的完整控制能力。

## 文档

- **[使用手册](docs/USAGE.md)** — CLI 命令行使用方法、C++ SDK 直接集成指南、配置说明、构建指南

## 快速开始

### Windows

```powershell
# 构建
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug

# 运行
.\build\Debug\moto_control.exe
```

### Ubuntu（22.04 / 24.04）

```bash
# 安装依赖
sudo apt install -y build-essential cmake git

# 构建
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)

# 运行前编辑串口配置（COM2 → /dev/ttyUSB0）
# vim assets/config.json

# 运行
./build/moto_control
```

进入 REPL 后输入 `help` 查看所有命令。

## 结构

```
MotorControl/
├── src/
│   ├── main.cpp              # CLI 入口
│   └── control/
│       ├── MotionControl.h   # 抽象接口
│       ├── ChuangRui_Control.h
│       └── ChuangRui_Control.cpp
├── include/                  # 头文件（json, modbus, log）
├── assets/config.json        # 配置文件
└── docs/USAGE.md             # 使用手册
```
