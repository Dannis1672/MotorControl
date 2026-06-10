#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <libmodbus/modbus.h>

/// 封装 libmodbus C API，提供：
///  - 连接生命周期管理 (RTU)
///  - 同步读写（带重试）
///  - 异步任务队列 + worker 线程
class ModbusClient {
public:
    // ── 类型定义 ──────────────────────────────

    struct Config {
        std::string device  = "COM2";
        int         baud    = 19200;
        char        parity  = 'N';
        int         data_bit = 8;
        int         stop_bit = 1;
        int         slave_id = 1;
    };

    struct Task {
        bool      is_write = false;   // false=read, true=write
        int       address  = 0;
        int       count    = 0;
        uint16_t* data     = nullptr; // 读：动态分配，worker 负责 delete[]
                                      // 写：指向持久内存，worker 不释放
    };

    /// 读任务完成回调。worker 在线程上下文中同步调用。
    /// 回调结束后 worker 会 delete[] data。
    using ReadCallback = std::function<void(int address, int count, uint16_t* data)>;

    // ── 构造 / 析构 ──────────────────────────

    ModbusClient() = default;
    ~ModbusClient();

    ModbusClient(const ModbusClient&)            = delete;
    ModbusClient& operator=(const ModbusClient&) = delete;

    // ── 生命周期 ──────────────────────────────

    /// 创建 RTU 上下文、设置从站、连接、配置超时。
    /// 成功返回 true，失败返回 false 并记录日志。
    bool connect(const Config& cfg);

    /// 断开并释放 modbus 上下文。
    void disconnect();

    // ── 同步读写 ──────────────────────────────

    /// 读取保持寄存器，失败时自动重试一次。
    /// 返回成功读取的寄存器数量，-1 表示最终失败。
    int readRegisters(int addr, int nb, uint16_t* dest);

    /// 写入多个保持寄存器，失败时自动重试一次。
    /// 返回写入的寄存器数量，-1 表示最终失败。
    int writeRegisters(int addr, int nb, const uint16_t* data);

    // ── 异步任务队列 ──────────────────────────

    /// 将任务推入队列并通知 worker。
    void pushTask(const Task& task);

    /// 设置读完成回调（必须在 startWorker 之前调用）。
    void setReadCallback(ReadCallback cb) { read_callback_ = std::move(cb); }

    // ── Worker 线程 ───────────────────────────

    void startWorker();
    void stopWorker();
    bool isWorkerRunning() const { return worker_running_.load(); }

    // ── 底层访问 ──────────────────────────────

    modbus_t* rawContext() const { return ctx_; }

private:
    void workerLoop();

    modbus_t*    ctx_ = nullptr;
    ReadCallback read_callback_;

    std::atomic<bool> worker_running_{false};
    std::thread        worker_thread_;

    std::mutex              queue_mutex_;
    std::queue<Task>        task_queue_;
    std::condition_variable queue_cv_;
};
