#include <mutex>
#include <thread>
#include "gtest/gtest.h"

#include "cyber/data/cache_buffer.h"

namespace apollo {
namespace cyber {
namespace data {

TEST(CacheBufferTest, cache_buffer_test) {
  CacheBuffer<int> buffer(32);
  EXPECT_TRUE(buffer.Empty());
  for (int i = 0; i < 32 - 1; i++) {
    buffer.Fill(std::move(i));
    EXPECT_FALSE(buffer.Full());
    EXPECT_EQ(i, buffer[i + 1]);
    EXPECT_EQ(i, buffer.at(i + 1));
  }
  EXPECT_EQ(31, buffer.Size());
  EXPECT_EQ(1, buffer.Head());
  EXPECT_EQ(31, buffer.Tail());
  EXPECT_EQ(0, buffer.Front());
  EXPECT_EQ(30, buffer.Back());
  buffer.Fill(31);
  EXPECT_TRUE(buffer.Full());
  EXPECT_EQ(32, buffer.Size());
  CacheBuffer<int> buffer1(std::move(buffer));
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
