
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/data/data_visitor.h"
#include "cyber/message/raw_message.h"

namespace apollo {
namespace cyber {
namespace data {

using apollo::cyber::message::RawMessage;
using apollo::cyber::proto::RoleAttributes;
std::hash<std::string> str_hash;

auto channel0 = str_hash("/channel0");
auto channel1 = str_hash("/channel1");
auto channel2 = str_hash("/channel2");
auto channel3 = str_hash("/channel3");

void DispatchMessage(uint64_t channel_id, int num) {
  for (int i = 0; i < num; ++i) {
    auto raw_msg = std::make_shared<RawMessage>();
    DataDispatcher<RawMessage>::Instance()->Dispatch(channel_id, raw_msg);
  }
}

std::vector<VisitorConfig> InitConfigs(int num) {
  std::vector<VisitorConfig> configs;
  for (int i = 0; i < num; ++i) {
    uint64_t channel_id = str_hash("/channel" + std::to_string(i));
    uint32_t queue_size = 10;
    configs.emplace_back(channel_id, queue_size);
  }
  return configs;
}

TEST(DataVisitorTest, one_channel) {
  auto channel0 = str_hash("/channel");
  auto dv = std::make_shared<DataVisitor<RawMessage>>(channel0, 10);

  DispatchMessage(channel0, 1);
  std::shared_ptr<RawMessage> msg;
  EXPECT_TRUE(dv->TryFetch(msg));
  EXPECT_FALSE(dv->TryFetch(msg));
  DispatchMessage(channel0, 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(dv->TryFetch(msg));
  }
  EXPECT_FALSE(dv->TryFetch(msg));
}

TEST(DataVisitorTest, two_channel) {
  auto dv =
      std::make_shared<DataVisitor<RawMessage, RawMessage>>(InitConfigs(2));

  DispatchMessage(channel0, 1);
  std::shared_ptr<RawMessage> msg0;
  std::shared_ptr<RawMessage> msg1;
  EXPECT_FALSE(dv->TryFetch(msg0, msg1));
  DispatchMessage(channel1, 1);
  EXPECT_TRUE(dv->TryFetch(msg0, msg1));
  DispatchMessage(channel0, 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(dv->TryFetch(msg0, msg1));
  }
  EXPECT_FALSE(dv->TryFetch(msg0, msg1));
}

TEST(DataVisitorTest, three_channel) {
  auto dv = std::make_shared<DataVisitor<RawMessage, RawMessage, RawMessage>>(
      InitConfigs(3));

  DispatchMessage(channel0, 1);
  std::shared_ptr<RawMessage> msg0;
  std::shared_ptr<RawMessage> msg1;
  std::shared_ptr<RawMessage> msg2;
  EXPECT_FALSE(dv->TryFetch(msg0, msg1, msg2));
  DispatchMessage(channel1, 1);
  EXPECT_FALSE(dv->TryFetch(msg0, msg1, msg2));
  DispatchMessage(channel2, 1);
  EXPECT_TRUE(dv->TryFetch(msg0, msg1, msg2));
  DispatchMessage(channel0, 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(dv->TryFetch(msg0, msg1, msg2));
  }
  EXPECT_FALSE(dv->TryFetch(msg0, msg1, msg2));
}

TEST(DataVisitorTest, four_channel) {
  auto dv = std::make_shared<
      DataVisitor<RawMessage, RawMessage, RawMessage, RawMessage>>(
      InitConfigs(4));

  DispatchMessage(channel0, 1);
  std::shared_ptr<RawMessage> msg0;
  std::shared_ptr<RawMessage> msg1;
  std::shared_ptr<RawMessage> msg2;
  std::shared_ptr<RawMessage> msg3;
  EXPECT_FALSE(dv->TryFetch(msg0, msg1, msg2, msg3));
  DispatchMessage(channel1, 1);
  EXPECT_FALSE(dv->TryFetch(msg0, msg1, msg2, msg3));
  DispatchMessage(channel2, 1);
  EXPECT_FALSE(dv->TryFetch(msg0, msg1, msg2, msg3));
  DispatchMessage(channel3, 1);
  EXPECT_TRUE(dv->TryFetch(msg0, msg1, msg2, msg3));
  DispatchMessage(channel0, 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(dv->TryFetch(msg0, msg1, msg2, msg3));
  }
  EXPECT_FALSE(dv->TryFetch(msg0, msg1, msg2, msg3));
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
