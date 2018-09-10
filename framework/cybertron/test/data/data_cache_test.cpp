#include <functional>
#include <mutex>
#include <thread>
#include "gtest/gtest.h"

#include "cybertron/common/log.h"
#include "cybertron/data/data_cache.h"
#include "cybertron/data/data_fusion.h"
#include "cybertron/message/raw_message.h"

namespace apollo {
namespace cybertron {
namespace data {

using apollo::cybertron::message::RawMessage;

std::hash<std::string> str_hash;

TEST(DataCacheTest, data_cache_test) {
  auto cache = DataCache::Instance();
  uint64_t channel_id_1 = str_hash("channel-1");
  uint64_t channel_id_2 = str_hash("channel-2");
  uint64_t channel_id_3 = str_hash("channel-3");

  cache->InitChannelCache(channel_id_1);
  cache->InitChannelCache(channel_id_1);
  std::vector<uint64_t> channel_vec(
      {channel_id_1, channel_id_2, channel_id_3});
  cache->InitChannelCache(channel_vec);

  // write data cache
  auto message = std::shared_ptr<RawMessage>(new RawMessage("0"));
  EXPECT_FALSE(
      cache->WriteDataCache<RawMessage>(channel_id_1, nullptr));
  EXPECT_TRUE(
      cache->WriteDataCache<RawMessage>(channel_id_1, message));
  for (int i = 1; i < DataCache::CHANNEL_CACHE_SIZE + 6; i++) {
    message = std::shared_ptr<RawMessage>(new RawMessage(std::to_string(i)));
    EXPECT_TRUE(
        cache->WriteDataCache<RawMessage>(channel_id_1, message));
  }

  DataFusionNotifier fusion;
  channel_vec.clear();
  channel_vec.push_back(channel_id_2);
  cache->RegisterFusionNotifier(fusion);
  EXPECT_TRUE(
      cache->WriteDataCache<RawMessage>(channel_id_2, message));

  // read data cache
  uint64_t seq_id = 0;
  uint64_t drop_num = 0;
  MetaDataPtr<RawMessage> read;
  EXPECT_FALSE(cache->ReadDataCache<RawMessage>(channel_id_3, 10,
                                                &seq_id, &drop_num, read));
  seq_id = 0;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, 10,
                                               &seq_id, &drop_num, read));
  EXPECT_EQ("69", read->message->message);

  DataCache::BufferValueType* channel_cache = nullptr;
  cache->channel_data_.Get(channel_id_1, &channel_cache);

  auto head = channel_cache->Head();
  auto tail = channel_cache->Tail();
  AINFO << "cache head: " << head << ", cache tail: " << tail;
  seq_id = 5;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, 10,
                                               &seq_id, &drop_num, read));
  EXPECT_EQ("6", read->message->message);
  EXPECT_EQ(2, drop_num);
  seq_id = 6;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, 10,
                                               &seq_id, &drop_num, read));
  EXPECT_EQ("6", read->message->message);
  EXPECT_EQ(3, drop_num);
  seq_id = tail;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, 10,
                                               &seq_id, &drop_num, read));
  EXPECT_EQ("69", read->message->message);
  seq_id = tail + 5;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, 10,
                                               &seq_id, &drop_num, read));
  EXPECT_EQ("10", read->message->message);
  seq_id = 32;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, 10,
                                               &seq_id, &drop_num, read));
  EXPECT_EQ("31", read->message->message);

  EXPECT_FALSE(
      cache->ReadDataCache<RawMessage>(channel_id_3, &seq_id, read));
  seq_id = 0;
  EXPECT_TRUE(
      cache->ReadDataCache<RawMessage>(channel_id_1, &seq_id, read));
  EXPECT_EQ("69", read->message->message);
  seq_id = 3;
  EXPECT_TRUE(
      cache->ReadDataCache<RawMessage>(channel_id_1, &seq_id, read));
  EXPECT_EQ("66", read->message->message);

  EXPECT_FALSE(cache->ReadDataCache<RawMessage>(channel_id_3, read));
  seq_id = 0;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, read));
  EXPECT_EQ("69", read->message->message);
  seq_id = 3;
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, read));
  EXPECT_EQ("69", read->message->message);

  std::vector<MetaDataPtr<RawMessage>> msg_vec;
  EXPECT_FALSE(
      cache->ReadDataCache<RawMessage>(channel_id_3, msg_vec));
  EXPECT_TRUE(cache->ReadDataCache<RawMessage>(channel_id_1, msg_vec));
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
