
#include "gtest/gtest.h"

#include "cyber/common/util.h"
#include "cyber/data/channel_buffer.h"

namespace apollo {
namespace cyber {
namespace data {

auto channel0 = common::Hash("/channel0");

TEST(ChannelBufferTest, Fetch) {
  auto cache_buffer = new CacheBuffer<std::shared_ptr<int>>(2);
  auto buffer = std::make_shared<ChannelBuffer<int>>(channel0, cache_buffer);
  std::shared_ptr<int> msg;
  uint64_t index = 0;
  EXPECT_FALSE(buffer->Fetch(&index, msg));
  buffer->Buffer()->Fill(std::make_shared<int>(1));
  EXPECT_TRUE(buffer->Fetch(&index, msg));
  EXPECT_EQ(1, *msg);
  EXPECT_EQ(1, index);
  index++;
  EXPECT_FALSE(buffer->Fetch(&index, msg));
  buffer->Buffer()->Fill(std::make_shared<int>(2));
  buffer->Buffer()->Fill(std::make_shared<int>(3));
  buffer->Buffer()->Fill(std::make_shared<int>(4));
  EXPECT_TRUE(buffer->Fetch(&index, msg));
  EXPECT_EQ(4, *msg);
  EXPECT_EQ(4, index);
  index++;
  EXPECT_FALSE(buffer->Fetch(&index, msg));
  EXPECT_EQ(4, *msg);
}

TEST(ChannelBufferTest, Latest) {
  auto cache_buffer = new CacheBuffer<std::shared_ptr<int>>(10);
  auto buffer = std::make_shared<ChannelBuffer<int>>(channel0, cache_buffer);
  std::shared_ptr<int> msg;
  uint64_t index = 0;
  EXPECT_FALSE(buffer->Latest(msg));

  buffer->Buffer()->Fill(std::make_shared<int>(1));
  EXPECT_TRUE(buffer->Latest(msg));
  EXPECT_EQ(1, *msg);
  EXPECT_TRUE(buffer->Latest(msg));
  EXPECT_EQ(1, *msg);

  buffer->Buffer()->Fill(std::make_shared<int>(2));
  EXPECT_TRUE(buffer->Latest(msg));
  EXPECT_EQ(2, *msg);
}

TEST(ChannelBufferTest, FetchMulti) {
  auto cache_buffer = new CacheBuffer<std::shared_ptr<int>>(2);
  auto buffer = std::make_shared<ChannelBuffer<int>>(channel0, cache_buffer);
  std::vector<std::shared_ptr<int>> vector;
  EXPECT_FALSE(buffer->FetchMulti(1, &vector));
  buffer->Buffer()->Fill(std::make_shared<int>(1));
  EXPECT_TRUE(buffer->FetchMulti(1, &vector));
  EXPECT_EQ(1, vector.size());
  EXPECT_EQ(1, *vector[0]);

  vector.clear();
  buffer->Buffer()->Fill(std::make_shared<int>(2));
  EXPECT_TRUE(buffer->FetchMulti(1, &vector));
  EXPECT_EQ(1, vector.size());
  EXPECT_EQ(2, *vector[0]);

  vector.clear();
  EXPECT_TRUE(buffer->FetchMulti(2, &vector));
  EXPECT_EQ(2, vector.size());
  EXPECT_EQ(1, *vector[0]);
  EXPECT_EQ(2, *vector[1]);

  vector.clear();
  EXPECT_TRUE(buffer->FetchMulti(3, &vector));
  EXPECT_EQ(2, vector.size());
  EXPECT_EQ(1, *vector[0]);
  EXPECT_EQ(2, *vector[1]);
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
