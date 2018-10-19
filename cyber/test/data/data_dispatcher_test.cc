
#include "gtest/gtest.h"

#include "cyber/common/util.h"
#include "cyber/data/data_dispatcher.h"

namespace apollo {
namespace cyber {
namespace data {

template <typename T>
using BufferVector =
    std::vector<std::weak_ptr<CacheBuffer<std::shared_ptr<T>>>>;

auto channel0 = common::Hash("/channel0");
auto channel1 = common::Hash("/channel1");

TEST(DataDispatcher, AddBuffer) {
  auto cache_buffer1 = new CacheBuffer<std::shared_ptr<int>>(2);
  auto buffer0 = ChannelBuffer<int>(channel0, cache_buffer1);
  auto cache_buffer2 = new CacheBuffer<std::shared_ptr<int>>(2);
  auto buffer1 = ChannelBuffer<int>(channel1, cache_buffer2);
  auto dispatcher = DataDispatcher<int>::Instance();
  dispatcher->AddBuffer(buffer0);
  dispatcher->AddBuffer(buffer1);

  BufferVector<int>* buffers = nullptr;
  EXPECT_TRUE(dispatcher->buffers_map_.Get(channel0, &buffers));
  EXPECT_EQ(1, buffers->size());
  EXPECT_TRUE(dispatcher->buffers_map_.Get(channel1, &buffers));
  EXPECT_EQ(1, buffers->size());
}

TEST(DataDispatcher, Dispatch) {
  auto cache_buffer = new CacheBuffer<std::shared_ptr<int>>(10);
  auto buffer = ChannelBuffer<int>(channel0, cache_buffer);
  auto dispatcher = DataDispatcher<int>::Instance();
  auto msg = std::make_shared<int>(1);

  EXPECT_FALSE(dispatcher->Dispatch(channel0, msg));
  dispatcher->AddBuffer(buffer);
  EXPECT_FALSE(dispatcher->Dispatch(channel0, msg));
  auto notifier = std::make_shared<Notifier>();
  DataNotifier::Instance()->AddNotifier(channel0, notifier);
  EXPECT_TRUE(dispatcher->Dispatch(channel0, msg));
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
