#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <vector>
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
        int                    address = 0;
        int                    count   = 0;
        std::vector<uint16_t>  data;

        /// 构造写任务：从 src 拷贝 count 个 uint16_t 到内部 vector。
        static Task makeWrite(int addr, int cnt, const uint16_t* src) {
            Task t;
            t.address = addr;
            t.count   = cnt;
            t.data.assign(src, src + cnt);
            return t;
        }
    };

    /// 读完成回调（周期性读取触发）。
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

    /// 设置失败重试等待间隔（毫秒），默认 100。
    void setRetryDelayMs(int ms) { retry_delay_ms_ = ms; }

    // ── Worker 线程 ───────────────────────────

    void startWorker();
    void stopWorker();
    bool isWorkerRunning() const { return worker_running_.load(); }

    // ── 周期性读取 ────────────────────────────

    /// 启用周期性读取：worker 每处理完一个任务后，
    /// 若距上次读取超过 interval_ms 毫秒则自动从 addr
    /// 读取 count 个寄存器。结果通过 ReadCallback 通知。
    void enablePeriodicRead(int addr, int count, int interval_ms);

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

    // ── 重试延迟 ──────────────────────────────

    int retry_delay_ms_ = 100;   // 失败后重试等待间隔

    // ── 周期性读取 ────────────────────────────
    bool periodic_read_enabled_ = false;
    int  periodic_read_addr_    = 0;
    int  periodic_read_count_   = 0;
    int  periodic_read_interval_ms_ = 200;
    std::chrono::steady_clock::time_point last_read_time_{};
};
