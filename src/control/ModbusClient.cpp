#include "control/ModbusClient.h"

#include <chrono>
#include <spdlog/spdlog.h>

ModbusClient::~ModbusClient() {
    stopWorker();
    disconnect();
}

// ── 生命周期 ────────────────────────────────────────────────

bool ModbusClient::connect(const Config& cfg) {
    if (ctx_) {
        spdlog::warn("ModbusClient: already connected, disconnecting first");
        disconnect();
    }

    ctx_ = modbus_new_rtu(cfg.device.c_str(), cfg.baud, cfg.parity,
                          cfg.data_bit, cfg.stop_bit);
    if (!ctx_) {
        spdlog::error("ModbusClient: modbus_new_rtu failed");
        return false;
    }

    if (modbus_set_slave(ctx_, cfg.slave_id) == -1) {
        spdlog::error("ModbusClient: modbus_set_slave({}) failed", cfg.slave_id);
    }

    if (modbus_connect(ctx_) == -1) {
        spdlog::error("ModbusClient: modbus_connect failed for {}", cfg.device);
    }

    // 默认超时
    modbus_set_byte_timeout(ctx_, 0, 100000);
    modbus_set_response_timeout(ctx_, 0, 100000);

    return true;
}

void ModbusClient::disconnect() {
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
}

// ── 同步读写 ────────────────────────────────────────────────

int ModbusClient::readRegisters(int addr, int nb, uint16_t* dest) {
    if (!ctx_) return -1;
    int ret = modbus_read_registers(ctx_, addr, nb, dest);
    if (ret == -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms_));
        ret = modbus_read_registers(ctx_, addr, nb, dest);
    }
    return ret;
}

int ModbusClient::writeRegisters(int addr, int nb, const uint16_t* data) {
    if (!ctx_) return -1;
    int ret = modbus_write_registers(ctx_, addr, nb, data);
    if (ret == -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms_));
        ret = modbus_write_registers(ctx_, addr, nb, data);
    }
    return ret;
}

// ── 异步任务队列 ────────────────────────────────────────────

void ModbusClient::pushTask(const Task& task) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.push(task);
    }
    queue_cv_.notify_one();
}

// ── 周期性读取 ────────────────────────────────────────────

void ModbusClient::enablePeriodicRead(int addr, int count, int interval_ms) {
    periodic_read_enabled_ = true;
    periodic_read_addr_    = addr;
    periodic_read_count_   = count;
    periodic_read_interval_ms_ = interval_ms;
    last_read_time_ = std::chrono::steady_clock::now();
}

// ── Worker 线程 ─────────────────────────────────────────────

void ModbusClient::startWorker() {
    if (worker_running_) return;
    worker_running_ = true;
    worker_thread_  = std::thread(&ModbusClient::workerLoop, this);
}

void ModbusClient::stopWorker() {
    if (!worker_running_) return;
    worker_running_ = false;
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    // 清空队列
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!task_queue_.empty()) {
        task_queue_.pop();
    }
}

void ModbusClient::workerLoop() {
    Task task;

    while (worker_running_ || !task_queue_.empty()) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !task_queue_.empty() || !worker_running_;
            });
            if (task_queue_.empty()) {
                if (!worker_running_) break;
                continue;
            }
            task = std::move(task_queue_.front());
            task_queue_.pop();
        }

        // ── 写任务 ───────────────────────────────
        int ret = modbus_write_registers(ctx_, task.address,
                                         task.count, task.data.data());
        if (ret == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms_));
            ret = modbus_write_registers(ctx_, task.address,
                                         task.count, task.data.data());
            if (ret == -1) {
                spdlog::error("ModbusClient: write error addr={} num={}",
                              task.address, task.count);
            }
        }

        // ── 周期性读取 ────────────────────────────
        if (periodic_read_enabled_) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_read_time_ >= std::chrono::milliseconds(periodic_read_interval_ms_)) {
                uint16_t buf[56];
                int ret = readRegisters(periodic_read_addr_,
                                        periodic_read_count_, buf);
                if (ret != -1 && read_callback_) {
                    read_callback_(periodic_read_addr_, periodic_read_count_, buf);
                }
                last_read_time_ = now;
            }
        }
    }
}
