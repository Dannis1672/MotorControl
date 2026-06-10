#pragma once

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "json.hpp"
#include "control/ChuangRui_Control.h"

namespace beast = boost::beast;
namespace http  = beast::http;
namespace net   = boost::asio;
using tcp       = net::ip::tcp;
using json      = nlohmann::json;

// ============================================================
// HTTP 服务器 — 将 ChuangRui_Control 的服务以 REST API 暴露
// ============================================================

class HttpServer {
public:
    HttpServer(ChuangRui_Control& ctrl, unsigned short port = 8080)
        : ctrl_(ctrl), port_(port) {}

    /// 在调用线程中阻塞运行（通常放在独立 std::thread 中）
    void run() {
        try {
            net::io_context ioc{1};
            tcp::acceptor acceptor{ioc, tcp::endpoint{tcp::v4(), port_}};
            spdlog::info("HTTP server listening on port {}", port_);

            while (true) {
                tcp::socket socket{ioc};
                acceptor.accept(socket);
                handle_session(std::move(socket));
            }
        } catch (const std::exception& e) {
            spdlog::error("HTTP server error: {}", e.what());
        }
    }

private:
    // ── 会话处理 ────────────────────────────────────

    void handle_session(tcp::socket socket) {
        beast::flat_buffer buffer;
        http::request<http::string_body> req;

        try {
            http::read(socket, buffer, req);
            auto res = handle_request(req);
            http::write(socket, res);
        } catch (const std::exception& e) {
            spdlog::warn("HTTP session error: {}", e.what());
        }

        beast::error_code ec;
        socket.shutdown(tcp::socket::shutdown_send, ec);
    }

    // ── 路由分发 ────────────────────────────────────

    http::response<http::string_body> handle_request(
        const http::request<http::string_body>& req)
    {
        auto const method = req.method();
        auto const target = std::string(req.target());
        spdlog::debug("HTTP {} {}", http::to_string(method), target);

        // 解析路径
        if (target == "/api/init" && method == http::verb::post) {
            return api_init();
        }
        if (target == "/api/free" && method == http::verb::post) {
            return api_free();
        }
        if (target == "/api/move" && method == http::verb::post) {
            return api_move(req.body());
        }
        if (target == "/api/tozero" && method == http::verb::post) {
            return api_tozero(req.body());
        }
        if (target == "/api/stop" && method == http::verb::post) {
            return api_stop(req.body());
        }
        if (target == "/api/bit" && method == http::verb::post) {
            return api_bit(req.body());
        }
        if (target == "/api/float" && method == http::verb::post) {
            return api_float(req.body());
        }
        if (target == "/api/state" && method == http::verb::get) {
            return api_state();
        }
        if (target == "/api/feed" && method == http::verb::post) {
            return api_feed(req.body());
        }
        if (target == "/api/begin" && method == http::verb::post) {
            return api_begin();
        }
        if (target == "/api/finish" && method == http::verb::post) {
            return api_finish();
        }

        // 404
        json err = {{"error", "not found"}, {"path", target}};
        return json_response(http::status::not_found, err);
    }

    // ── API 处理器 ──────────────────────────────────

    http::response<http::string_body> api_init() {
        auto r = ctrl_.ControlInitial();
        json resp;
        resp["result"] = (r == MotionControl::Result::Success) ? "success" : "failure";
        return json_response(http::status::ok, resp);
    }

    http::response<http::string_body> api_free() {
        auto r = ctrl_.ControlFree();
        json resp;
        resp["result"] = (r == MotionControl::Result::Success) ? "success" : "failure";
        return json_response(http::status::ok, resp);
    }

    http::response<http::string_body> api_move(const std::string& body) {
        try {
            auto j = json::parse(body);
            auto axis = parse_axis(j.at("axis").get<std::string>());
            float dist = j.at("distance").get<float>();
            auto r = ctrl_.AxisMove(axis, dist);
            json resp;
            resp["result"] = (r == MotionControl::Result::Success) ? "success" : "failure";
            return json_response(http::status::ok, resp);
        } catch (const std::exception& e) {
            return error_response("move: " + std::string(e.what()));
        }
    }

    http::response<http::string_body> api_tozero(const std::string& body) {
        try {
            auto j = json::parse(body);
            auto axis = parse_axis(j.at("axis").get<std::string>());
            auto r = ctrl_.AxisToZero(axis);
            json resp;
            resp["result"] = (r == MotionControl::Result::Success) ? "success" : "failure";
            return json_response(http::status::ok, resp);
        } catch (const std::exception& e) {
            return error_response("tozero: " + std::string(e.what()));
        }
    }

    http::response<http::string_body> api_stop(const std::string& body) {
        try {
            auto j = json::parse(body);
            auto axis = parse_axis(j.at("axis").get<std::string>());
            auto r = ctrl_.AxisStop(axis);
            json resp;
            resp["result"] = (r == MotionControl::Result::Success) ? "success" : "failure";
            return json_response(http::status::ok, resp);
        } catch (const std::exception& e) {
            return error_response("stop: " + std::string(e.what()));
        }
    }

    http::response<http::string_body> api_bit(const std::string& body) {
        try {
            auto j = json::parse(body);
            auto btn = parse_button(j.at("button").get<std::string>());
            bool val = j.at("value").get<int>() != 0;
            auto r = ctrl_.WriteBit(btn, val);
            json resp;
            resp["result"] = (r == MotionControl::Result::Success) ? "success" : "failure";
            return json_response(http::status::ok, resp);
        } catch (const std::exception& e) {
            return error_response("bit: " + std::string(e.what()));
        }
    }

    http::response<http::string_body> api_float(const std::string& body) {
        try {
            auto j = json::parse(body);
            auto param = parse_float_param(j.at("param").get<std::string>());
            float val = j.at("value").get<float>();
            auto r = ctrl_.WriteFloat(param, val);
            json resp;
            resp["result"] = (r == MotionControl::Result::Success) ? "success" : "failure";
            return json_response(http::status::ok, resp);
        } catch (const std::exception& e) {
            return error_response("float: " + std::string(e.what()));
        }
    }

    http::response<http::string_body> api_state() {
        json state = ctrl_.GetSystemState();
        return json_response(http::status::ok, state);
    }

    http::response<http::string_body> api_feed(const std::string& body) {
        try {
            auto j = json::parse(body);
            float zd  = j.at("zd").get<float>();
            float fup = j.at("fup").get<float>();
            bool ok = ctrl_.IsFeed(zd, fup);
            json resp;
            resp["complete"] = ok;
            return json_response(http::status::ok, resp);
        } catch (const std::exception& e) {
            return error_response("feed: " + std::string(e.what()));
        }
    }

    http::response<http::string_body> api_begin() {
        ctrl_.ProcessBegin();
        json resp{{"result", "ok"}};
        return json_response(http::status::ok, resp);
    }

    http::response<http::string_body> api_finish() {
        ctrl_.ProcessFinish();
        json resp{{"result", "ok"}};
        return json_response(http::status::ok, resp);
    }

    // ── 辅助函数 ────────────────────────────────────

    static http::response<http::string_body> json_response(
        http::status status, const json& body)
    {
        http::response<http::string_body> res{status, 11};
        res.set(http::field::server, "moto_control");
        res.set(http::field::content_type, "application/json");
        res.keep_alive(false);
        res.body() = body.dump();
        res.prepare_payload();
        return res;
    }

    static http::response<http::string_body> error_response(const std::string& msg) {
        json err{{"error", msg}};
        return json_response(http::status::bad_request, err);
    }

    // ── 参数解析（与 main.cpp 中的 CommandLine 保持一致）──

    static MotionControl::Axis parse_axis(const std::string& s) {
        if (s == "Z" || s == "z") return MotionControl::Axis::Z;
        if (s == "F" || s == "f") return MotionControl::Axis::F;
        if (s == "C" || s == "c") return MotionControl::Axis::C;
        throw std::runtime_error("unknown axis: " + s);
    }

    static MotionControl::UIButton parse_button(const std::string& s) {
        static const std::map<std::string, MotionControl::UIButton> tbl = {
            {"Light", MotionControl::Light},
            {"ReSet", MotionControl::ReSet},
            {"Door", MotionControl::Door},
            {"GasCharge", MotionControl::GasCharge},
            {"LaserEnabled", MotionControl::LaserEnabled},
            {"BackFlush", MotionControl::BackFlush},
            {"CharmberPressureTest", MotionControl::CharmberPressureTest},
            {"WholePressureTest", MotionControl::WholePressureTest},
            {"Ventilate", MotionControl::Ventilate},
            {"ProcessMode", MotionControl::ProcessMode},
            {"LaserCooler", MotionControl::LaserCooler},
            {"LaserPower", MotionControl::LaserPower},
            {"LaserIndicate", MotionControl::LaserIndicate},
            {"PowderFeed", MotionControl::PowderFeed},
            {"Feed", MotionControl::Feed},
            {"CLeft", MotionControl::CLeft},
            {"CRight", MotionControl::CRight},
            {"Fup", MotionControl::Fup},
            {"FDown", MotionControl::FDown},
            {"MotorPower", MotionControl::MotorPower},
        };
        auto it = tbl.find(s);
        if (it != tbl.end()) return it->second;
        throw std::runtime_error("unknown button: " + s);
    }

    static MotionControl::UIFloat parse_float_param(const std::string& s) {
        static const std::map<std::string, MotionControl::UIFloat> tbl = {
            {"CharmberPressure", MotionControl::CharmberPressure},
            {"PressureDown", MotionControl::PressureDown},
            {"OxygenRatio", MotionControl::OxygenRatio},
            {"OxygenRatioDown", MotionControl::OxygenRatioDown},
            {"WindSpeed", MotionControl::WindSpeed},
            {"windPressure", MotionControl::windPressure},
            {"Temperature", MotionControl::Temperature},
        };
        auto it = tbl.find(s);
        if (it != tbl.end()) return it->second;
        throw std::runtime_error("unknown float param: " + s);
    }

    // ── 成员 ────────────────────────────────────────

    ChuangRui_Control& ctrl_;
    unsigned short      port_;
};
