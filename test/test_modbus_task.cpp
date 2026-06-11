/// ModbusClient::Task 创建与数据测试
#include <gtest/gtest.h>
#include <cstring>
#include "control/ModbusClient.h"

// ── makeWrite 基本功能：地址与计数 ──
TEST(ModbusTask, MakeWrite_AddressAndCount) {
    uint16_t src[3] = {100, 200, 300};
    auto task = ModbusClient::Task::makeWrite(42, 3, src);

    EXPECT_EQ(task.address, 42);
    EXPECT_EQ(task.count, 3);
}

// ── makeWrite 数据拷贝正确性 ──
TEST(ModbusTask, MakeWrite_DataCopy) {
    uint16_t src[4] = {0x1234, 0x5678, 0x9ABC, 0xDEF0};
    auto task = ModbusClient::Task::makeWrite(10, 4, src);

    ASSERT_EQ(task.data.size(), 4u);
    EXPECT_EQ(task.data[0], 0x1234u);
    EXPECT_EQ(task.data[1], 0x5678u);
    EXPECT_EQ(task.data[2], 0x9ABCu);
    EXPECT_EQ(task.data[3], 0xDEF0u);
}

// ── makeWrite 零计数 ──
TEST(ModbusTask, MakeWrite_ZeroCount) {
    uint16_t src[1] = {42};
    auto task = ModbusClient::Task::makeWrite(0, 0, src);

    EXPECT_EQ(task.address, 0);
    EXPECT_EQ(task.count, 0);
    EXPECT_TRUE(task.data.empty());
}

// ── makeWrite 单寄存器 ──
TEST(ModbusTask, MakeWrite_SingleRegister) {
    uint16_t val = 0xABCD;
    auto task = ModbusClient::Task::makeWrite(10, 1, &val);

    EXPECT_EQ(task.address, 10);
    EXPECT_EQ(task.count, 1);
    ASSERT_EQ(task.data.size(), 1u);
    EXPECT_EQ(task.data[0], 0xABCDu);
}

// ── 默认构造的 Task ──
TEST(ModbusTask, DefaultConstructor) {
    ModbusClient::Task task;
    EXPECT_EQ(task.address, 0);
    EXPECT_EQ(task.count, 0);
    EXPECT_TRUE(task.data.empty());
}
