#include <gtest/gtest.h>

#include <chrono>

#include "autorunner_driver/protocol/assembler.hpp"

namespace proto = autorunner::protocol;
using Clock = proto::TripletAssembler::Clock;
using std::chrono::milliseconds;

TEST(AssemblerTest, InOrderComplete)
{
  proto::TripletAssembler asm_;
  const auto t0 = Clock::now();
  EXPECT_FALSE(asm_.feed(0, 1.0, 2.0, t0).has_value());
  EXPECT_FALSE(asm_.feed(1, 3.0, 4.0, t0).has_value());
  const auto full = asm_.feed(2, 5.0, 6.0, t0);
  ASSERT_TRUE(full.has_value());
  EXPECT_EQ((*full)[0], 1.0);
  EXPECT_EQ((*full)[1], 2.0);
  EXPECT_EQ((*full)[2], 3.0);
  EXPECT_EQ((*full)[3], 4.0);
  EXPECT_EQ((*full)[4], 5.0);
  EXPECT_EQ((*full)[5], 6.0);
}

TEST(AssemblerTest, OutOfOrderComplete)
{
  proto::TripletAssembler asm_;
  const auto t0 = Clock::now();
  // part0 之后乱序到达 part2/part1
  EXPECT_FALSE(asm_.feed(0, 1.0, 2.0, t0).has_value());
  EXPECT_FALSE(asm_.feed(2, 5.0, 6.0, t0).has_value());
  EXPECT_TRUE(asm_.feed(1, 3.0, 4.0, t0).has_value());
}

TEST(AssemblerTest, Part0ResetsCycle)
{
  proto::TripletAssembler asm_;
  const auto t0 = Clock::now();
  EXPECT_FALSE(asm_.feed(0, 1.0, 2.0, t0).has_value());
  EXPECT_FALSE(asm_.feed(1, 3.0, 4.0, t0).has_value());
  // 新的 part0 到达: 重置, 旧 part1 作废
  EXPECT_FALSE(asm_.feed(0, 10.0, 20.0, t0).has_value());
  EXPECT_FALSE(asm_.feed(1, 30.0, 40.0, t0).has_value());
  const auto full = asm_.feed(2, 50.0, 60.0, t0);
  ASSERT_TRUE(full.has_value());
  EXPECT_EQ((*full)[0], 10.0);
  EXPECT_EQ((*full)[2], 30.0);
}

TEST(AssemblerTest, IncompleteNoOutput)
{
  proto::TripletAssembler asm_;
  const auto t0 = Clock::now();
  EXPECT_FALSE(asm_.feed(0, 1.0, 2.0, t0).has_value());
  EXPECT_FALSE(asm_.feed(1, 3.0, 4.0, t0).has_value());
  // 缺 part2 -> 无输出
}

TEST(AssemblerTest, TimeoutDiscardsStale)
{
  proto::TripletAssembler asm_(milliseconds(50));
  const auto t0 = Clock::now();
  EXPECT_FALSE(asm_.feed(1, 3.0, 4.0, t0).has_value());
  EXPECT_FALSE(asm_.feed(2, 5.0, 6.0, t0).has_value());
  // 60ms 后来了 part1/part2, 旧半包已超时丢弃, 不会与新 part0 错拼
  const auto t1 = t0 + milliseconds(60);
  EXPECT_FALSE(asm_.feed(1, 30.0, 40.0, t1).has_value());
  EXPECT_FALSE(asm_.feed(2, 50.0, 60.0, t1).has_value());
  EXPECT_FALSE(asm_.feed(0, 10.0, 20.0, t1).has_value());  // part0 重置后重新开始
  EXPECT_FALSE(asm_.feed(1, 31.0, 41.0, t1).has_value());
  const auto full = asm_.feed(2, 51.0, 61.0, t1);
  ASSERT_TRUE(full.has_value());
  EXPECT_EQ((*full)[0], 10.0);
  EXPECT_EQ((*full)[2], 31.0);
  EXPECT_EQ((*full)[4], 51.0);
}

TEST(AssemblerTest, InvalidPartIgnored)
{
  proto::TripletAssembler asm_;
  EXPECT_FALSE(asm_.feed(3, 1.0, 2.0).has_value());
}

TEST(AssemblerTest, SnapshotUpdate)
{
  proto::DriverHighSpeedSnapshot snap;
  proto::DriverHighSpeedFb fb;
  fb.joint_index = 2;
  fb.speed = 1.5;
  fb.current = 2.0;
  fb.position = 0.5;
  snap.update(fb);
  EXPECT_EQ(snap.speed[2], 1.5);
  EXPECT_EQ(snap.current[2], 2.0);
  EXPECT_EQ(snap.position[2], 0.5);
  EXPECT_EQ(snap.speed[0], 0.0);

  // 越界 joint_index 忽略
  fb.joint_index = 6;
  fb.speed = 99.0;
  snap.update(fb);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NE(snap.speed[i], 99.0);
  }
}
