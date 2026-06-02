#ifndef STREAM_PROCESSOR_H
#define STREAM_PROCESSOR_H

#include <functional>
#include <vector>
#include <mutex>
#include <array>
#include <string>
enum class ExceptionLevel {
    Debug = 0,
    Info,
    Warning,
    Error,
    Fatal
};

class StreamProcessor {
public:
    // 回调类型定义
    using ErrorCallback = std::function<void(const std::string&)>;

    StreamProcessor() : _closed(false) {}

    // 禁止拷贝（流一般不拷贝）
    StreamProcessor(const StreamProcessor&) = delete;
    StreamProcessor& operator=(const StreamProcessor&) = delete;

    // 允许移动
    StreamProcessor(StreamProcessor&&) = default;
    StreamProcessor& operator=(StreamProcessor&&) = default;

    // 注册异常回调，按等级分组（可注册多个）
    void onCallback(ExceptionLevel level, ErrorCallback cb) {
        std::lock_guard<std::mutex> lock(_mtx);
        auto idx = static_cast<std::size_t>(level);
        if (idx >= _errorCallbacks.size()) return;
        _errorCallbacks[idx].push_back(std::move(cb));
    }

    // 推送异常（按等级分发）
    void push(ExceptionLevel level, const std::string& msg) {
        std::lock_guard<std::mutex> lock(_mtx);
        if (_closed) return;
        auto idx = static_cast<std::size_t>(level);
        if (idx >= _errorCallbacks.size()) return;
        for (const auto& cb : _errorCallbacks[idx]) {
            if (cb) cb(msg);
        }
    }

    // 清空所有回调
    void clear() {
        std::lock_guard<std::mutex> lock(_mtx);
        for (auto &v : _errorCallbacks) v.clear();
    }

    bool isClosed() const {
        std::lock_guard<std::mutex> lock(_mtx);
        return _closed;
    }

private:
    mutable std::mutex _mtx;
    bool _closed;
    std::vector<std::vector<ErrorCallback>> _unused_holder; // placeholder to keep layout stable if needed
    static constexpr std::size_t kExceptionLevelCount = static_cast<std::size_t>(ExceptionLevel::Fatal) + 1;
    std::array<std::vector<ErrorCallback>, kExceptionLevelCount> _errorCallbacks;
};

#endif