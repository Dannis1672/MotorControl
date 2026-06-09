#pragma once

// moto_log.h — 基于 spdlog 的多 sink 日志工具
// 支持: 控制台（彩色）、文件（滚动）、TCP 远程传输
// 配置来源: config.json → "logging" 段
//
// 用法:
//   #include "moto_log.h"
//   nlohmann::json cfg = ...加载 config.json...
//   moto::log::init(cfg);
//   spdlog::info("hello");            // 零修改，自动路由到多 sink
//   moto::log::shutdown();            // 程序退出前调用

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/tcp_sink.h>

#include <memory>
#include <string>
#include <vector>

#include "json.hpp"

namespace moto {
namespace log {

// ===================================================================
// 配置数据结构
// ===================================================================

struct ConsoleSinkConfig {
    bool enabled = true;
    bool color   = true;
};

struct FileSinkConfig {
    bool        enabled        = false;
    std::string path           = "logs/moto_control.log";
    size_t      max_size       = 10 * 1024 * 1024;   // 10 MiB
    size_t      max_files      = 5;
    bool        rotate_on_open = false;
};

struct TcpSinkConfig {
    bool        enabled       = false;
    std::string host          = "127.0.0.1";
    int         port          = 11000;
    bool        lazy_connect  = true;
};

struct LogConfig {
    spdlog::level::level_enum level            = spdlog::level::info;
    bool                      async_enabled    = true;
    size_t                    async_queue_size = 8192;
    size_t                    async_threads    = 1;
    std::string               pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";
    ConsoleSinkConfig         console;
    FileSinkConfig            file;
    TcpSinkConfig             tcp;
};

// ===================================================================
// JSON 解析辅助
// ===================================================================

inline spdlog::level::level_enum parse_level(const std::string& s) {
    if (s == "trace")    return spdlog::level::trace;
    if (s == "debug")    return spdlog::level::debug;
    if (s == "info")     return spdlog::level::info;
    if (s == "warn")     return spdlog::level::warn;
    if (s == "error")    return spdlog::level::err;
    if (s == "critical") return spdlog::level::critical;
    return spdlog::level::info;
}

inline LogConfig load_log_config(const nlohmann::json& cfg) {
    LogConfig lc;
    if (!cfg.contains("logging")) return lc;

    const auto& L = cfg["logging"];

    // 日志等级
    if (L.contains("level"))
        lc.level = parse_level(L["level"].get<std::string>());

    // 异步配置
    if (L.contains("async"))
        lc.async_enabled = L["async"].get<bool>();
    if (L.contains("async_queue_size"))
        lc.async_queue_size = L["async_queue_size"].get<size_t>();
    if (L.contains("async_threads"))
        lc.async_threads = L["async_threads"].get<size_t>();

    // 格式模板
    if (L.contains("pattern"))
        lc.pattern = L["pattern"].get<std::string>();

    // sink 列表
    if (L.contains("sinks") && L["sinks"].is_array()) {
        for (const auto& sink : L["sinks"]) {
            std::string type = sink.value("type", "");
            if (type == "console") {
                lc.console.enabled = sink.value("enabled", true);
                lc.console.color   = sink.value("color",   true);
            }
            else if (type == "file") {
                lc.file.enabled        = sink.value("enabled",        false);
                lc.file.path           = sink.value("path",           lc.file.path);
                lc.file.max_size       = sink.value("max_size",       lc.file.max_size);
                lc.file.max_files      = sink.value("max_files",      lc.file.max_files);
                lc.file.rotate_on_open = sink.value("rotate_on_open", false);
            }
            else if (type == "tcp") {
                lc.tcp.enabled      = sink.value("enabled",      false);
                lc.tcp.host         = sink.value("host",         lc.tcp.host);
                lc.tcp.port         = sink.value("port",         lc.tcp.port);
                lc.tcp.lazy_connect = sink.value("lazy_connect", true);
            }
        }
    }
    return lc;
}

// ===================================================================
// 核心初始化 / 关闭
// ===================================================================

inline void init(const nlohmann::json& config_json) {
    auto cfg = load_log_config(config_json);

    std::vector<spdlog::sink_ptr> sinks;

    // -- 1. 控制台 sink --
    if (cfg.console.enabled) {
        if (cfg.console.color) {
            sinks.push_back(
                std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        } else {
            sinks.push_back(
                std::make_shared<spdlog::sinks::stdout_sink_mt>());
        }
    }

    // -- 2. 滚动文件 sink --
    if (cfg.file.enabled) {
        try {
            sinks.push_back(
                std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                    cfg.file.path,
                    cfg.file.max_size,
                    cfg.file.max_files,
                    cfg.file.rotate_on_open));
        } catch (const spdlog::spdlog_ex& e) {
            // 文件 sink 创建失败（如路径不可写）→ 回退控制台输出错误并继续
            if (sinks.empty()) {
                sinks.push_back(
                    std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
            }
            auto fallback = std::make_shared<spdlog::logger>(
                "fallback", sinks.front());
            fallback->error("Failed to create file sink: {}", e.what());
        }
    }

    // -- 3. TCP sink --
    if (cfg.tcp.enabled) {
        try {
            spdlog::sinks::tcp_sink_config tcp_cfg(
                cfg.tcp.host, cfg.tcp.port);
            tcp_cfg.lazy_connect = cfg.tcp.lazy_connect;
            sinks.push_back(
                std::make_shared<spdlog::sinks::tcp_sink_mt>(
                    std::move(tcp_cfg)));
        } catch (const spdlog::spdlog_ex& e) {
            if (sinks.empty()) {
                sinks.push_back(
                    std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
            }
            auto fallback = std::make_shared<spdlog::logger>(
                "fallback", sinks.front());
            fallback->error("Failed to create TCP sink: {}", e.what());
        }
    }

    // -- 4. 兜底: 至少一个 sink --
    if (sinks.empty()) {
        sinks.push_back(
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }

    // -- 5. 构建 logger --
    std::shared_ptr<spdlog::logger> logger;

    if (cfg.async_enabled) {
        spdlog::init_thread_pool(cfg.async_queue_size, cfg.async_threads);
        auto tp = spdlog::thread_pool();
        logger = std::make_shared<spdlog::async_logger>(
            "moto_control",
            sinks.begin(), sinks.end(),
            tp,
            spdlog::async_overflow_policy::block);
    } else {
        logger = std::make_shared<spdlog::logger>(
            "moto_control",
            sinks.begin(), sinks.end());
    }

    logger->set_level(cfg.level);
    logger->set_pattern(cfg.pattern);
    logger->flush_on(spdlog::level::err);   // error/critical 自动 flush

    spdlog::set_default_logger(logger);
}

inline void shutdown() {
    spdlog::shutdown();
}

} // namespace log
} // namespace moto
