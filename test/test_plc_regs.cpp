/// CharmRayPlcRegs 结构体测试
/// 验证：总大小、位域布局、成员偏移量
#include <gtest/gtest.h>
#include <cstring>
#include "control/CharmRayPlcRegs.h"

// ── 总大小：56 个寄存器 × 2 字节 = 112 字节 ──
TEST(CharmRayPlcRegs, TotalSize) {
    EXPECT_EQ(sizeof(CharmRayPlcRegs), 112u);
}

// ── MW10 位域：16 位全部占满 ──
TEST(CharmRayPlcRegs, MW10_AllBits) {
    CharmRayPlcRegs regs{};
    uint16_t raw;

    // 逐位置 1 → 检查
    regs.MW10.security = 1;
    std::memcpy(&raw, &regs.MW10, sizeof(raw));
    EXPECT_EQ(raw & 1, 1u);

    regs.MW10.light = 1;
    std::memcpy(&raw, &regs.MW10, sizeof(raw));
    EXPECT_EQ((raw >> 1) & 1, 1u);

    regs.MW10.laser_on = 1;
    std::memcpy(&raw, &regs.MW10, sizeof(raw));
    EXPECT_EQ((raw >> 14) & 1, 1u);

    regs.MW10.fan_on = 1;
    std::memcpy(&raw, &regs.MW10, sizeof(raw));
    EXPECT_EQ((raw >> 15) & 1, 1u);
}

// ── MW11 位域测试：回零和运动指令 ──
TEST(CharmRayPlcRegs, MW11_HomeAndMoveBits) {
    CharmRayPlcRegs regs{};
    uint16_t raw;

    // 同时设置 Z 轴回零指令和 Z 轴运动开始指令
    regs.MW11.Z轴回原点指令 = 1;
    regs.MW11.Z轴运动开始指令 = 1;
    std::memcpy(&raw, &regs.MW11, sizeof(raw));
    EXPECT_EQ(raw & 1, 1u);       // bit 0
    EXPECT_EQ((raw >> 3) & 1, 1u); // bit 3

    // C 轴
    regs.MW11.C轴回原点指令 = 1;
    regs.MW11.C轴运动开始指令 = 1;
    std::memcpy(&raw, &regs.MW11, sizeof(raw));
    EXPECT_EQ((raw >> 2) & 1, 1u); // bit 2
    EXPECT_EQ((raw >> 5) & 1, 1u); // bit 5
}

// ── MW13_MW14 状态位：急停、腔门、原点标志 ──
TEST(CharmRayPlcRegs, MW13_MW14_StatusBits) {
    CharmRayPlcRegs regs{};
    uint16_t raw;

    // MW13 位：急停安全继电器
    regs.MW13_MW14.急停安全继电器状态 = 1;
    std::memcpy(&raw, &regs.MW13_MW14, sizeof(raw));
    EXPECT_EQ((raw >> 3) & 1, 1u);

    // MW14 位：C 轴原点标志
    regs.MW13_MW14.C轴原点标志 = 1;
    std::memcpy(&raw, &regs.MW13_MW14, sizeof(raw));
    EXPECT_EQ((raw >> 11) & 1, 1u);
}

// ── MW12_12_14 报警和限位信号 ──
TEST(CharmRayPlcRegs, MW12_12_14_AlarmLimitBits) {
    CharmRayPlcRegs regs{};
    uint16_t raw;

    regs.MW12_12_14.Z轴驱动器报警 = 1;
    regs.MW12_12_14.Z轴运动超时报警 = 1;
    std::memcpy(&raw, &regs.MW12_12_14, sizeof(raw));
    EXPECT_EQ(raw & 1, 1u);
    EXPECT_EQ((raw >> 1) & 1, 1u);

    regs.MW12_12_14.Z轴正限位 = 1;
    regs.MW12_12_14.Z轴负限位 = 1;
    std::memcpy(&raw, &regs.MW12_12_14, sizeof(raw));
    EXPECT_EQ((raw >> 6) & 1, 1u);
    EXPECT_EQ((raw >> 9) & 1, 1u);

    regs.MW12_12_14.铺粉到位 = 1;
    std::memcpy(&raw, &regs.MW12_12_14, sizeof(raw));
    EXPECT_EQ((raw >> 15) & 1, 1u);
}

// ── 运动参数读写：int32_t 字段 ──
TEST(CharmRayPlcRegs, MotionParamFields) {
    CharmRayPlcRegs regs{};

    regs.Z轴加速度 = 60000;      // 12.0 * 5000
    regs.Z轴运动速度 = 15000;    // 3.0 * 5000
    regs.Z轴移动距离 = 2500;     // 0.5 * 5000
    EXPECT_EQ(regs.Z轴加速度, 60000);
    EXPECT_EQ(regs.Z轴运动速度, 15000);
    EXPECT_EQ(regs.Z轴移动距离, 2500);
}

// ── 当前位置字段 ──
TEST(CharmRayPlcRegs, CurrentPositionFields) {
    CharmRayPlcRegs regs{};

    regs.Z轴当前位置 = -1000;
    regs.F轴当前位置 = 5000;
    regs.C轴当前位置 = 32000;
    EXPECT_EQ(regs.Z轴当前位置, -1000);
    EXPECT_EQ(regs.F轴当前位置, 5000);
    EXPECT_EQ(regs.C轴当前位置, 32000);
}

// ── 零初始化后所有位域为 0 ──
TEST(CharmRayPlcRegs, ZeroInit) {
    CharmRayPlcRegs regs{};
    uint8_t bytes[sizeof(regs)] = {};
    std::memcpy(bytes, &regs, sizeof(regs));

    // 全零 → 所有位域为 0
    for (size_t i = 0; i < sizeof(regs); ++i) {
        EXPECT_EQ(bytes[i], 0u) << "byte " << i << " not zero";
    }
}

// ── 位域独立不干扰 ──
TEST(CharmRayPlcRegs, BitIndependence) {
    CharmRayPlcRegs regs{};

    // 设置 security=1 不应影响 light
    regs.MW10.security = 1;
    EXPECT_EQ(static_cast<int>(regs.MW10.light), 0);
    EXPECT_EQ(static_cast<int>(regs.MW10.security), 1);
}
