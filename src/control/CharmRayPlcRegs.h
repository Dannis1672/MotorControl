#pragma once

#include <cstdint>

/// CharmRay PLC 保持寄存器映射（Modbus 地址 40011-40066，共 56 个寄存器 = 112 字节）。
/// 2 字节对齐以匹配 PLC 的寄存器布局。
#pragma pack(push, 2)
struct CharmRayPlcRegs {
    // ── Modbus 地址 40011 ──
    struct MW10 {
        unsigned short security       : 1;   // 0
        unsigned short light          : 1;   // 1
        unsigned short red_light      : 1;   // 2
        unsigned short greeen_light   : 1;   // 3
        unsigned short blue_light     : 1;   // 4
        unsigned short alarm          : 1;   // 5
        unsigned short big_valve      : 1;   // 6
        unsigned short small_valve    : 1;   // 7
        unsigned short ventilate      : 1;   // 8
        unsigned short lase_power     : 1;   // 9
        unsigned short fan_power      : 1;   // 10
        unsigned short motor_power    : 1;   // 11
        unsigned short laser_indicator: 1;   // 12
        unsigned short laser_enabled  : 1;   // 13
        unsigned short laser_on       : 1;   // 14
        unsigned short fan_on         : 1;   // 15
    } MW10;

    // ── Modbus 地址 40012 ──
    struct MW11 {
        unsigned short Z轴回原点指令   : 1;   // 0
        unsigned short F轴回原点指令   : 1;   // 1
        unsigned short C轴回原点指令   : 1;   // 2
        unsigned short Z轴运动开始指令 : 1;   // 3
        unsigned short F轴运动开始指令 : 1;   // 4
        unsigned short C轴运动开始指令 : 1;   // 5
        unsigned short unused_6_15     : 10;  // 6-15
    } MW11;

    // ── Modbus 地址 40013-40014 ──
    struct MW13_MW14 {
        // MW13
        unsigned short 大充气阀状态       : 1;   // 0
        unsigned short 小充气阀状态       : 1;   // 1
        unsigned short 排气阀状态         : 1;   // 2
        unsigned short 急停安全继电器状态 : 1;   // 3
        unsigned short 腔门插到位信号     : 1;   // 4
        unsigned short 腔门锁上电状态     : 1;   // 5
        unsigned short 激光器有电信号     : 1;   // 6
        unsigned short 风机有电信号       : 1;   // 7
        unsigned short 电机有电信号       : 1;   // 8
        // MW14
        unsigned short Z轴原点标志        : 1;   // 9
        unsigned short F轴原点标志        : 1;   // 10
        unsigned short C轴原点标志        : 1;   // 11
        unsigned short unused_12_15       : 4;   // 12-15
    } MW13_MW14;

    // ── Modbus 地址 40015 ──
    struct MW12 {
        unsigned short 激光器报警 : 1;          // 0
        unsigned short 风机报警   : 1;          // 1
        unsigned short unused_2_15: 14;         // 2-15
    } MW12;

    // ── Modbus 地址 40016 ──
    struct MW12_12_14 {
        unsigned short Z轴驱动器报警   : 1;     // 0
        unsigned short Z轴运动超时报警 : 1;     // 1
        unsigned short F轴驱动器报警   : 1;     // 2
        unsigned short F轴运动超时报警 : 1;     // 3
        unsigned short C轴驱动器报警   : 1;     // 4
        unsigned short C轴运动超时报警 : 1;     // 5
        unsigned short Z轴正限位       : 1;     // 6
        unsigned short F轴正限位       : 1;     // 7
        unsigned short C轴正限位       : 1;     // 8
        unsigned short Z轴负限位       : 1;     // 9
        unsigned short F轴负限位       : 1;     // 10
        unsigned short C轴负限位       : 1;     // 11
        unsigned short Z轴运动结束信号 : 1;     // 12
        unsigned short F轴运动结束信号 : 1;     // 13
        unsigned short C轴运动结束信号 : 1;     // 14
        unsigned short 铺粉到位         : 1;     // 15
    } MW12_12_14;

    // ── Modbus 地址 40017 ──
    struct STOP {
        unsigned short Z轴运动停止信号 : 1;     // 0
        unsigned short F轴运动停止信号 : 1;     // 1
        unsigned short C轴运动停止信号 : 1;     // 2
        unsigned short unused_3_15     : 13;    // 3-15
    } STOP;

    // ── Modbus 地址 40017-40020 (unused) ──
    uint16_t unused_17_20[4];

    // ── Modbus 地址 40021-40038 ──
    int32_t Z轴加速度;
    int32_t Z轴运动速度;
    int32_t Z轴移动距离;

    int32_t F轴加速度;
    int32_t F轴运动速度;
    int32_t F轴移动距离;

    int32_t C轴加速度;
    int32_t C轴运动速度;
    int32_t C轴移动距离;

    // ── Modbus 地址 40039-40040 (unused) ──
    int16_t unused_39_40[2];

    // ── Modbus 地址 40041-40043 ──
    int16_t 风压设定值;
    int16_t 压力预警设定值;
    int16_t 压力报警设定值;

    // ── Modbus 地址 40044-40050 (unused) ──
    uint16_t usused44_50[7];

    // ── Modbus 地址 40051-40054 ──
    int16_t 氧含量低精度;
    int16_t 氧含量高精度;
    int16_t 风压实际值;
    int16_t 腔体压力;

    // ── Modbus 地址 40055-40060 (unused) ──
    uint16_t usused55_60[6];

    // ── Modbus 地址 40061-40066 ──
    int32_t Z轴当前位置;
    int32_t F轴当前位置;
    int32_t C轴当前位置;
};
#pragma pack(pop)

// 编译期断言：56 个寄存器 × 2 字节 = 112 字节
static_assert(sizeof(CharmRayPlcRegs) == 112,
              "CharmRayPlcRegs must be 112 bytes (56 registers)");
