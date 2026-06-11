/// ChuangRui_Control::GetSystemState 测试
/// 验证 JSON 输出的正确性：字段存在、类型、值
#include <gtest/gtest.h>
#include <cmath>
#include "json.hpp"
#include "control/ChuangRui_Control.h"

using json = nlohmann::json;

class GetSystemStateTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 填充一组已知的寄存器值
        CharmRayPlcRegs regs{};
        regs.MW10.light = 1;
        regs.MW10.alarm = 1;
        regs.MW10.motor_power = 1;

        regs.MW11.Z轴回原点指令 = 1;
        regs.MW11.C轴运动开始指令 = 1;

        regs.MW13_MW14.急停安全继电器状态 = 1;
        regs.MW13_MW14.Z轴原点标志 = 1;

        regs.MW12_12_14.Z轴驱动器报警 = 1;
        regs.MW12_12_14.铺粉到位 = 1;

        regs.STOP.C轴运动停止信号 = 1;

        regs.风压设定值 = 100;
        regs.压力预警设定值 = 80;
        regs.压力报警设定值 = 120;
        regs.风压实际值 = 95;
        regs.腔体压力 = 50;
        regs.氧含量低精度 = 180;
        regs.氧含量高精度 = 220;

        regs.Z轴加速度 = 60000;
        regs.Z轴运动速度 = 15000;
        regs.Z轴移动距离 = 2500;
        regs.F轴加速度 = 60000;
        regs.F轴运动速度 = 15000;
        regs.F轴移动距离 = 5000;
        regs.C轴加速度 = 33320;
        regs.C轴运动速度 = 14280;
        regs.C轴移动距离 = 1904;

        regs.Z轴当前位置 = -100;
        regs.F轴当前位置 = 200;
        regs.C轴当前位置 = 32000;

        ChuangRui_Control::TestSetRegisters(regs);
    }

    json state_;

    void FetchState() {
        ChuangRui_Control ctrl;
        state_ = ctrl.GetSystemState();
    }
};

// ── 顶层字段存在性 ──
TEST_F(GetSystemStateTest, TopLevelSectionsExist) {
    FetchState();
    EXPECT_TRUE(state_.contains("MW10"));
    EXPECT_TRUE(state_.contains("MW11"));
    EXPECT_TRUE(state_.contains("MW13_MW14"));
    EXPECT_TRUE(state_.contains("MW12"));
    EXPECT_TRUE(state_.contains("MW12_12_14"));
    EXPECT_TRUE(state_.contains("STOP"));
}

// ── MW10 位域值 ──
TEST_F(GetSystemStateTest, MW10_Values) {
    FetchState();
    auto& mw10 = state_["MW10"];
    EXPECT_EQ(mw10["light"].get<int>(), 1);
    EXPECT_EQ(mw10["alarm"].get<int>(), 1);
    EXPECT_EQ(mw10["motor_power"].get<int>(), 1);
    // 未设置的位
    EXPECT_EQ(mw10["security"].get<int>(), 0);
    EXPECT_EQ(mw10["big_valve"].get<int>(), 0);
}

// ── MW11 位域值 ──
TEST_F(GetSystemStateTest, MW11_Values) {
    FetchState();
    auto& mw11 = state_["MW11"];
    EXPECT_EQ(mw11["Z轴回原点指令"].get<int>(), 1);
    EXPECT_EQ(mw11["C轴运动开始指令"].get<int>(), 1);
    EXPECT_EQ(mw11["F轴回原点指令"].get<int>(), 0);
}

// ── MW13_MW14 状态值 ──
TEST_F(GetSystemStateTest, MW13_MW14_Values) {
    FetchState();
    auto& m = state_["MW13_MW14"];
    EXPECT_EQ(m["急停安全继电器状态"].get<int>(), 1);
    EXPECT_EQ(m["Z轴原点标志"].get<int>(), 1);
    EXPECT_EQ(m["大充气阀状态"].get<int>(), 0);
}

// ── MW12_12_14 报警/限位 ──
TEST_F(GetSystemStateTest, MW12_12_14_Values) {
    FetchState();
    auto& m = state_["MW12_12_14"];
    EXPECT_EQ(m["Z轴驱动器报警"].get<int>(), 1);
    EXPECT_EQ(m["铺粉到位"].get<int>(), 1);
    EXPECT_EQ(m["F轴驱动器报警"].get<int>(), 0);
}

// ── STOP 信号 ──
TEST_F(GetSystemStateTest, STOP_Values) {
    FetchState();
    auto& s = state_["STOP"];
    EXPECT_EQ(s["C轴运动停止信号"].get<int>(), 1);
    EXPECT_EQ(s["Z轴运动停止信号"].get<int>(), 0);
}

// ── 数值型字段 ──
TEST_F(GetSystemStateTest, NumericValues) {
    FetchState();
    EXPECT_EQ(state_["风压设定值"].get<int>(), 100);
    EXPECT_EQ(state_["压力预警设定值"].get<int>(), 80);
    EXPECT_EQ(state_["压力报警设定值"].get<int>(), 120);
    EXPECT_EQ(state_["风压实际值"].get<int>(), 95);
    EXPECT_EQ(state_["腔体压力"].get<int>(), 50);
    EXPECT_EQ(state_["氧含量低精度"].get<int>(), 180);
    EXPECT_EQ(state_["氧含量高精度"].get<int>(), 220);
}

// ── 运动参数 ──
TEST_F(GetSystemStateTest, MotionParams) {
    FetchState();
    EXPECT_EQ(state_["Z轴加速度"].get<int>(), 60000);
    EXPECT_EQ(state_["Z轴运动速度"].get<int>(), 15000);
    EXPECT_EQ(state_["Z轴移动距离"].get<int>(), 2500);
    EXPECT_EQ(state_["F轴加速度"].get<int>(), 60000);
    EXPECT_EQ(state_["F轴运动速度"].get<int>(), 15000);
    EXPECT_EQ(state_["F轴移动距离"].get<int>(), 5000);
    EXPECT_EQ(state_["C轴加速度"].get<int>(), 33320);
    EXPECT_EQ(state_["C轴运动速度"].get<int>(), 14280);
    EXPECT_EQ(state_["C轴移动距离"].get<int>(), 1904);
}

// ── 当前位置 ──
TEST_F(GetSystemStateTest, CurrentPositions) {
    FetchState();
    EXPECT_EQ(state_["Z轴当前位置"].get<int>(), -100);
    EXPECT_EQ(state_["F轴当前位置"].get<int>(), 200);
    EXPECT_EQ(state_["C轴当前位置"].get<int>(), 32000);
}

// ── GetSystemState 非空 ──
TEST_F(GetSystemStateTest, OutputIsNotEmpty) {
    FetchState();
    EXPECT_FALSE(state_.empty());
    EXPECT_TRUE(state_.is_object());
}

// ── 运动控制参数验证 ──
TEST(MotionControlParams, DefaultValues) {
    // 验证默认值在 ControlInitial 之前
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetMoveDistance(0), 0.5f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetAccelerate(0), 12.0f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetVelocity(0), 3.0f);

    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetMoveDistance(1), 1.0f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetAccelerate(1), 12.0f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetVelocity(1), 3.0f);

    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetMoveDistance(2), 20.0f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetAccelerate(2), 350.0f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetVelocity(2), 150.0f);
}

// ── 电机比率验证 ──
TEST(MotionControlParams, MotorRatios) {
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetMotorRatio(0), 5000.0f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetMotorRatio(1), 5000.0f);
    EXPECT_FLOAT_EQ(ChuangRui_Control::TestGetMotorRatio(2), 95.2f);
}
