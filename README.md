# moto_control

基于 Modbus RTU 的增材制造设备运动控制软件，封装创瑞 PLC 的完整控制能力。

支持两种运行模式：**交互式 CLI** 和 **HTTP REST API**。

## 文档

- **[使用手册](docs/USAGE.md)** — CLI / HTTP API 使用方法、C++ SDK 集成、配置说明、构建指南

## 快速开始

### Ubuntu（22.04 / 24.04）

```bash
# 安装依赖
sudo apt install -y build-essential cmake git libboost-system-dev libboost-dev

# 构建
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)

# CLI 模式
./build/moto_control

# HTTP 服务器模式
./build/moto_control --http 8080
```

### Windows

```powershell
# 构建
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug

# 运行
.\build\Debug\moto_control.exe
```

## HTTP API 速览

启动 HTTP 模式后，所有控制操作通过 REST API 访问：

| 方法 | 路径 | 功能 |
|------|------|------|
| POST | `/api/init` | 初始化控制资源 |
| POST | `/api/free` | 释放控制资源 |
| POST | `/api/move` | 轴移动 `{"axis":"Z","distance":10.5}` |
| POST | `/api/tozero` | 轴回零 `{"axis":"Z"}` |
| POST | `/api/stop` | 轴停止 `{"axis":"Z"}` |
| POST | `/api/bit` | 写数字量 `{"button":"Light","value":1}` |
| POST | `/api/float` | 写浮点参数 `{"param":"OxygenRatio","value":0.25}` |
| GET  | `/api/state` | 系统状态 JSON |
| POST | `/api/feed` | 送粉检查 `{"zd":0.5,"fup":1.0}` |
| POST | `/api/begin` | 开始加工 |
| POST | `/api/finish` | 结束加工 |

详细参数见 [使用手册 - HTTP API](docs/USAGE.md#http-rest-api)。

## 结构

```
MotorControl/
├── src/
│   ├── main.cpp              # CLI / HTTP 入口
│   ├── control/
│   │   ├── MotionControl.h   # 抽象接口
│   │   ├── ChuangRui_Control.h / .cpp
│   │   └── ModbusClient.h / .cpp
│   └── server/
│       └── HttpServer.h      # HTTP REST API 服务器
├── include/                  # 头文件（json, modbus, log）
├── assets/config.json        # 配置文件
└── docs/USAGE.md             # 使用手册
```

## 依赖

| 库 | 用途 |
|---|---|
| libmodbus（内置） | Modbus RTU 通信 |
| spdlog（FetchContent） | 日志 |
| Boost.System + Beast/Asio | HTTP 服务器 |
| nlohmann/json（内置） | JSON 解析 |

