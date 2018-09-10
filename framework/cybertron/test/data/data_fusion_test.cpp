#include <mutex>
#include <thread>
#include "gtest/gtest.h"

#include "cybertron/message/raw_message.h"
#include "cybertron/data/data_fusion.h"
#include "cybertron/data/meta_data.h"

namespace apollo {
namespace cybertron {
namespace data {

using apollo::cybertron::message::RawMessage;

static MetaMsgVec<RawMessage> early_msg1_vec;
static MetaMsgVec<RawMessage> late_msg1_vec;
static MetaMsgVec<RawMessage> equal_msg1_vec;
static MetaMsgVec<RawMessage> fusion_msg1_vec;

static MetaMsgVec<RawMessage> early_msg2_vec;
static MetaMsgVec<RawMessage> late_msg2_vec;
static MetaMsgVec<RawMessage> equal_msg2_vec;
static MetaMsgVec<RawMessage> fusion_msg2_vec;

static MetaMsgVec<RawMessage> early_msg3_vec;
static MetaMsgVec<RawMessage> late_msg3_vec;
static MetaMsgVec<RawMessage> equal_msg3_vec;
static MetaMsgVec<RawMessage> fusion_msg3_vec;

static MetaDataPtr<RawMessage> msg0;
static MetaDataPtr<RawMessage> msg1;
static MetaDataPtr<RawMessage> msg2;
static MetaDataPtr<RawMessage> msg3;

std::hash<std::string> str_hash;

static void init_vecs() {
  auto message = std::shared_ptr<RawMessage>(new RawMessage("data"));
  msg0 = std::make_shared<MetaData<RawMessage>>();
  msg0->channel_id = str_hash("channel-1");
  msg0->time_stamp = 1000 * 1000 * 1000;  // 1s
  msg0->message = message;

  auto msg = std::make_shared<MetaData<RawMessage>>();
  uint32_t start = 1000 * 1000 * 1000 - 100 * 1000 * 1000 / 2;
  uint32_t end = 1000 * 1000 * 1000 + 100 * 1000 * 1000 / 2;
  for (int i = 0; i < 5; i++) {
    msg = std::make_shared<MetaData<RawMessage>>();
    msg->channel_id = str_hash("channel-2");
    msg->time_stamp = start - i * 50 * 1000 * 1000;  // 1s
    msg->message = message;
    early_msg1_vec.push_back(msg);
  }

  msg = std::make_shared<MetaData<RawMessage>>();
  msg->channel_id = str_hash("channel-2");
  msg->time_stamp = start + 50 * 1000 * 1000;  // 1s
  msg->message = message;
  fusion_msg1_vec.push_back(msg);

  msg = std::make_shared<MetaData<RawMessage>>();
  msg->channel_id = str_hash("channel-2");
  msg->time_stamp = end;  // 1s
  msg->message = message;
  equal_msg1_vec.push_back(msg);

  for (int i = 0; i < 5; i++) {
    msg = std::make_shared<MetaData<RawMessage>>();
    msg->channel_id = str_hash("channel-2");
    msg->time_stamp = end + (i + 1) * 50 * 1000 * 1000;  // 1s
    msg->message = message;
    late_msg1_vec.push_back(msg);
  }

  early_msg2_vec = early_msg1_vec;
  early_msg3_vec = early_msg2_vec;

  late_msg2_vec = late_msg1_vec;
  late_msg3_vec = late_msg1_vec;

  equal_msg2_vec = equal_msg1_vec;
  equal_msg3_vec = equal_msg1_vec;

  fusion_msg2_vec = fusion_msg1_vec;
  fusion_msg3_vec = fusion_msg1_vec;
}

TEST(DataFusionTest, channel_fusion_test) {
  init_vecs();
  DataFusion fusion;
  bool ret = fusion.process<RawMessage, RawMessage>(msg0, msg1, early_msg1_vec);
  EXPECT_FALSE(ret);
  ret = fusion.process<RawMessage, RawMessage>(msg0, msg1, late_msg1_vec);
  EXPECT_FALSE(ret);
  ret = fusion.process<RawMessage, RawMessage>(msg0, msg1, equal_msg1_vec);
  EXPECT_TRUE(ret);
  ret = fusion.process<RawMessage, RawMessage>(msg0, msg1, fusion_msg1_vec);
  EXPECT_TRUE(ret);

  ret = fusion.process<RawMessage, RawMessage, RawMessage>(
      msg0, msg1, early_msg1_vec, msg2, early_msg2_vec);
  EXPECT_FALSE(ret);
  ret = fusion.process<RawMessage, RawMessage, RawMessage>(
      msg0, msg1, fusion_msg1_vec, msg2, early_msg2_vec);
  EXPECT_FALSE(ret);
  ret = fusion.process<RawMessage, RawMessage, RawMessage>(
      msg0, msg1, fusion_msg1_vec, msg2, fusion_msg2_vec);
  EXPECT_TRUE(ret);

  ret = fusion.process<RawMessage, RawMessage, RawMessage, RawMessage>(
      msg0, msg1, early_msg1_vec, msg2, early_msg2_vec, msg3, early_msg3_vec);
  EXPECT_FALSE(ret);
  ret = fusion.process<RawMessage, RawMessage, RawMessage, RawMessage>(
      msg0, msg1, fusion_msg1_vec, msg2, early_msg2_vec, msg3, early_msg3_vec);
  EXPECT_FALSE(ret);
  ret = fusion.process<RawMessage, RawMessage, RawMessage, RawMessage>(
      msg0, msg1, fusion_msg1_vec, msg2, fusion_msg2_vec, msg3, early_msg3_vec);
  EXPECT_FALSE(ret);
  ret = fusion.process<RawMessage, RawMessage, RawMessage, RawMessage>(
      msg0, msg1, fusion_msg1_vec, msg2, fusion_msg2_vec, msg3,
      fusion_msg3_vec);
  EXPECT_TRUE(ret);
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
