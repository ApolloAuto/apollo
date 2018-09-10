#include <thread>
#include <mutex>
#include "gtest/gtest.h"

#include "cybertron/data/cache_buffer.h"

namespace apollo {
namespace cybertron {
namespace data {

TEST(CacheBufferTest, cache_buffer_test) {
  CacheBuffer<int, 32> buffer;
  EXPECT_TRUE(buffer.Empty());
  for (int i = 0; i < 32 - 1 - 1; i++) {
    buffer.Fill(std::move(i));
    EXPECT_FALSE(buffer.Full());
  }
  EXPECT_EQ(30, buffer.Size());
  EXPECT_EQ(0, buffer.Head());
  EXPECT_EQ(30, buffer.Tail());
  EXPECT_EQ(1, buffer.Front());
  EXPECT_EQ(29, buffer.Back());
  EXPECT_EQ(1, buffer[0]);
  EXPECT_EQ(15, buffer.at(16));
  buffer.Fill(31);
  EXPECT_TRUE(buffer.Full());
  EXPECT_EQ(31, buffer.Size());
  auto& rw_lock = buffer.RWLock();
  CacheBuffer<int, 32> buffer1(std::move(buffer));
}


}  // namespace data
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
