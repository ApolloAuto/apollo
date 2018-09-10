#include "gtest/gtest.h"
#include <iostream>
#include "cybertron/time/time.h"
#include "cybertron/time/duration.h"

namespace apollo {
namespace cybertron {

TEST(TimeTest, constructor) {
  Time time(100UL);
  EXPECT_EQ(100UL, time.ToNanosecond());

  time = Time(1.1);
  EXPECT_EQ(1100000000UL, time.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.1, time.ToSecond());

  time = Time(1, 1);
  EXPECT_EQ(1000000001UL, time.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.000000001, time.ToSecond());

  Time time2(time);
  EXPECT_TRUE(time == time2);
}

TEST(TimeTest, operators) {
  Time t1(100);
  Duration d(200);
  Time t2(300);
  EXPECT_TRUE(t1 != t2);
  EXPECT_TRUE(t1 < t2);
  EXPECT_TRUE(t1 <= t2);
  EXPECT_TRUE(t2 > t1);
  EXPECT_TRUE(t2 >= t1);
  EXPECT_TRUE(t1 + d == t2);
  EXPECT_TRUE(t2 - d == t1);
  EXPECT_TRUE((t1 += d) == t2);
  EXPECT_TRUE(t1 >= t2);
  EXPECT_TRUE(t1 <= t2);
  EXPECT_TRUE(Time(100) == (t1 -= d));
}

TEST(TimeTest, to_string) {
  Time t1(1531225311123456789UL);
  EXPECT_STREQ("2018-07-10 20:21:51.123456789", t1.ToString().c_str());
}

TEST(TimeTest, now) {
  std::cout << "Time Now: " << Time::Now() << std::endl;
}

TEST(TimeTest, is_zero) {
  Time time;
  EXPECT_TRUE(time.IsZero());
  EXPECT_FALSE(Time::MAX.IsZero());
  EXPECT_FALSE(Time::MIN.IsZero());
}

}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}