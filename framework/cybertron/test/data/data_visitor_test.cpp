
#include "gtest/gtest.h"

#include "cybertron/common/log.h"
#include "cybertron/cybertron.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/message/raw_message.h"

namespace apollo {
namespace cybertron {
namespace data {

using apollo::cybertron::message::RawMessage;
using apollo::cybertron::proto::RoleAttributes;
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

std::vector<std::shared_ptr<ReaderBase>> InitReaders(int num) {
  cybertron::Init();
  auto node = CreateNode("data_visitor_test");
  std::vector<std::shared_ptr<ReaderBase>> readers;
  for (int i = 0; i < num; ++i) {
    RoleAttributes attr;
    attr.set_channel_name("/channel" + std::to_string(i));
    auto qos_profile = attr.mutable_qos_profile();
    qos_profile->set_depth(10);
    auto reader = node->CreateReader<RawMessage>(attr);
    readers.emplace_back(reader);
  }
  return readers;
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
      std::make_shared<DataVisitor<RawMessage, RawMessage>>(InitReaders(2));

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
      InitReaders(3));

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
      InitReaders(4));

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
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
