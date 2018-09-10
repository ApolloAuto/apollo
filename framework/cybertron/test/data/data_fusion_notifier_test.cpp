#include "gtest/gtest.h"

#include "cybertron/common/log.h"
#include "cybertron/data/data_cache.h"
#include "cybertron/data/data_fusion_notifier.h"

namespace apollo {
namespace cybertron {
namespace data {

bool notified = false;
std::hash<std::string> str_hash;

auto channel_0 = str_hash("channel-0");
auto channel_1 = str_hash("channel-1");
auto channel_2 = str_hash("channel-2");
auto channel_3 = str_hash("channel-3");
auto data_cache = DataCache::Instance();

std::vector<uint64_t> channels = {channel_0, channel_1, channel_2, channel_3};
std::vector<uint64_t> channels_other = {channel_0, channel_1, channel_2};

TEST(DataFusionNotifierTest, status) {
  DataFusionNotifier notifier;
  notifier.RegisterCallback(channels, []() {});
  EXPECT_EQ(channel_0, notifier.MainChannel());
  EXPECT_FALSE(notifier.all_has_msg_);
  for (auto& status : notifier.channel_status_) {
    EXPECT_FALSE(status.has_msg);
  }
}

TEST(DataFusionNotifierTest, notify) {
  bool notified = false;
  bool other_notified = false;
  DataFusionNotifier notifier;
  DataFusionNotifier notifier_other;
  notifier.RegisterCallback(channels, [&notified]() { notified = true; });
  notifier_other.RegisterCallback(
      channels_other, [&other_notified]() { other_notified = true; });
  data_cache->InitChannelCache(channels);
  data_cache->RegisterFusionNotifier(notifier);
  data_cache->RegisterFusionNotifier(notifier_other);

  data_cache->WriteDataCache(channel_0, std::make_shared<int>(1));
  EXPECT_FALSE(notifier.UpdateChannelStatus());
  EXPECT_FALSE(notified);
  EXPECT_FALSE(notifier_other.UpdateChannelStatus());
  EXPECT_FALSE(other_notified);

  data_cache->WriteDataCache(channel_1, std::make_shared<int>(1));
  EXPECT_FALSE(notifier.UpdateChannelStatus());
  EXPECT_FALSE(notified);
  EXPECT_FALSE(notifier_other.UpdateChannelStatus());
  EXPECT_FALSE(other_notified);

  data_cache->WriteDataCache(channel_0, std::make_shared<int>(1));
  EXPECT_FALSE(notifier.UpdateChannelStatus());
  EXPECT_FALSE(notified);
  EXPECT_FALSE(notifier_other.UpdateChannelStatus());
  EXPECT_FALSE(other_notified);

  data_cache->WriteDataCache(channel_2, std::make_shared<int>(1));
  EXPECT_FALSE(notifier.UpdateChannelStatus());
  EXPECT_FALSE(notified);
  AINFO << "other";
  EXPECT_TRUE(notifier_other.UpdateChannelStatus());
  EXPECT_FALSE(other_notified);

  data_cache->WriteDataCache(channel_3, std::make_shared<int>(1));
  EXPECT_TRUE(notifier.UpdateChannelStatus());
  EXPECT_FALSE(notified);

  data_cache->WriteDataCache(channel_0, std::make_shared<int>(1));
  EXPECT_TRUE(notified);
  EXPECT_TRUE(other_notified);
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
