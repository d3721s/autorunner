#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "autorunner_driver/protocol/endian.hpp"

namespace proto = autorunner::protocol;

TEST(EndianTest, I32RoundTrip)
{
  uint8_t buf[4];
  for (int32_t v : {0, 1, -1, 90000, -100000,
      std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::min()})
  {
    proto::put_i32_be(buf, v);
    EXPECT_EQ(proto::get_i32_be(buf), v);
  }
}

TEST(EndianTest, I16RoundTrip)
{
  uint8_t buf[2];
  for (int16_t v : {int16_t{0}, int16_t{1}, int16_t{-1},
      std::numeric_limits<int16_t>::max(), std::numeric_limits<int16_t>::min()})
  {
    proto::put_i16_be(buf, v);
    EXPECT_EQ(proto::get_i16_be(buf), v);
  }
}

TEST(EndianTest, U16RoundTrip)
{
  uint8_t buf[2];
  for (uint16_t v : {uint16_t{0}, uint16_t{1}, uint16_t{0x7FFF}, uint16_t{0xFFFF}}) {
    proto::put_u16_be(buf, v);
    EXPECT_EQ(proto::get_u16_be(buf), v);
  }
}

TEST(EndianTest, BigEndianByteOrder)
{
  // 90000 = 0x00015F90, 大端: {00, 01, 5F, 90}
  uint8_t buf[4];
  proto::put_i32_be(buf, 90000);
  EXPECT_EQ(buf[0], 0x00);
  EXPECT_EQ(buf[1], 0x01);
  EXPECT_EQ(buf[2], 0x5F);
  EXPECT_EQ(buf[3], 0x90);

  // -100000 = 0xFFFE7960
  proto::put_i32_be(buf, -100000);
  EXPECT_EQ(buf[0], 0xFF);
  EXPECT_EQ(buf[1], 0xFE);
  EXPECT_EQ(buf[2], 0x79);
  EXPECT_EQ(buf[3], 0x60);
}

TEST(EndianTest, NanToInvalidSentinel)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(proto::to_fixed_i16_or_invalid(nan, proto::kDeciDeg), proto::kInvalid16);
  EXPECT_EQ(
    proto::to_fixed_u16_or_invalid(nan, proto::kCentiRad),
    static_cast<uint16_t>(proto::kInvalid16));
  // 非 NaN 正常缩放
  EXPECT_EQ(proto::to_fixed_u16_or_invalid(1.0, proto::kCentiRad), 100);
}

TEST(EndianTest, ScaleConstants)
{
  // 90° = 1.5708rad -> 0.001° 定点应为 90000
  EXPECT_EQ(proto::to_fixed_i32(proto::kPi / 2.0, proto::kMilliDeg), 90000);
  // 0.1m -> 0.001mm 定点应为 100000
  EXPECT_EQ(proto::to_fixed_i32(0.1, proto::kMicroMeter), 100000);
}
