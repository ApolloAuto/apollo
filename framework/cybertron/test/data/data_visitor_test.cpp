#include <mutex>
#include <thread>
#include "gtest/gtest.h"

#include "cybertron/common/log.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/data/any.h"
#include "cybertron/data/data_visitor.h"

namespace apollo {
namespace cybertron {
namespace data {

using apollo::cybertron::message::RawMessage;

std::hash<std::string> str_hash;

static std::vector<uint64_t> channel_vec({str_hash("channel-0"),
                                          str_hash("channel-1"),
                                          str_hash("channel-2"),
                                          str_hash("channel-3")});
static std::function<void()> callback = []() {
  AINFO << "callback ";
};

static void fill_msg0(DataVisitor& visitor) {
  auto message = std::shared_ptr<RawMessage>(new RawMessage(std::to_string(0)));
  visitor.data_cache_->WriteDataCache<RawMessage>(str_hash("channel-0"),
                                                  message);
  uint64_t seq_id = 0;
  uint64_t drop_num = 0;
  MetaDataPtr<RawMessage> msg;
  visitor.data_cache_->ReadDataCache(str_hash("channel-0"), 10, &seq_id,
                                     &drop_num, msg);
  msg->time_stamp = 1000 * 1000 * 1000;
}

static void fill_msg(DataVisitor& visitor, uint64_t channel) {
  auto message = std::shared_ptr<RawMessage>(new RawMessage(std::to_string(0)));
  AINFO << "fill_msg: " << channel;
  visitor.data_cache_->WriteDataCache<RawMessage>(channel, message);
  message = std::shared_ptr<RawMessage>(new RawMessage(std::to_string(1)));
  visitor.data_cache_->WriteDataCache<RawMessage>(channel, message);
  message = std::shared_ptr<RawMessage>(new RawMessage(std::to_string(2)));
  visitor.data_cache_->WriteDataCache<RawMessage>(channel, message);
}

static void make_msg_fusioned(DataVisitor& visitor, uint64_t channel) {
  AINFO << "make_msg_fusioned: " << channel;
  MetaDataPtr<RawMessage> msg;
  visitor.data_cache_->ReadDataCache(channel, msg);
  msg->time_stamp = 1000 * 1000 * 1000;
  visitor.data_cache_->ReadDataCache(channel, msg);
  AINFO << msg->time_stamp;
}

TEST(DataVisitorTest, one_channel_test) {
  auto channels = channel_vec;
  DataVisitor newest_visitor(std::move(channels), 1);
  AINFO << channels.size();
  channels = channel_vec;
  DataVisitor callback_visitor(std::move(channels), 1);
  auto cb = callback;
  callback_visitor.RegisterCallback(std::move(cb));
  std::shared_ptr<RawMessage> message;
  EXPECT_FALSE(newest_visitor.TryFetch<RawMessage>(message));
  EXPECT_FALSE(callback_visitor.TryFetch<RawMessage>(message));
  fill_msg0(newest_visitor);
  EXPECT_TRUE(newest_visitor.TryFetch<RawMessage>(message));
  EXPECT_EQ("0", message->message);
}

TEST(DataVisitorTest, two_channel_test) {
  auto channels = channel_vec;
  DataVisitor newest_visitor(std::move(channels), 1);
  channels = channel_vec;
  DataVisitor window_visitor(std::move(channels), 1,
                             FusionType::TIME_WINDOW);
  channels = channel_vec;
  DataVisitor callback_visitor(std::move(channels), 1);
  auto cb = callback;
  callback_visitor.RegisterCallback(std::move(cb));
  std::shared_ptr<RawMessage> msg0;
  std::shared_ptr<RawMessage> msg1;

  // empty
  bool ret;
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_FALSE(ret);
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_FALSE(ret);
  ret = callback_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_FALSE(ret);
  // fill msg0, but msg1's cache is still empty
  fill_msg0(newest_visitor);
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_FALSE(ret);
  fill_msg0(window_visitor);
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_FALSE(ret);

  // fill msg1 with now timestamp, not fusioned
  fill_msg(newest_visitor, str_hash("channel-1"));
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_TRUE(ret);
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_FALSE(ret);
  // fusioned
  make_msg_fusioned(window_visitor, str_hash("channel-1"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1);
  EXPECT_TRUE(ret);
}

TEST(DataVisitorTest, three_channel_test) {
  auto channels = channel_vec;
  DataVisitor newest_visitor(std::move(channels), 1);
  channels = channel_vec;
  DataVisitor window_visitor(std::move(channels), 1,
                             FusionType::TIME_WINDOW);
  channels = channel_vec;
  DataVisitor callback_visitor(std::move(channels), 1);
  auto cb = callback;
  callback_visitor.RegisterCallback(std::move(cb));
  std::shared_ptr<RawMessage> msg0;
  std::shared_ptr<RawMessage> msg1;
  std::shared_ptr<RawMessage> msg2;

  // empty
  bool ret;
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);
  ret = callback_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);
  // fill msg0, but msg1 and msg2's cache is still empty
  fill_msg0(newest_visitor);
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);
  fill_msg0(window_visitor);
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);
  // fill msg1, but msg2's cache is still empty
  fill_msg(newest_visitor, str_hash("channel-1"));
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);
  fill_msg(window_visitor, str_hash("channel-1"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);

  // fill msg2, but not fusioned
  fill_msg(newest_visitor, str_hash("channel-2"));
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_TRUE(ret);
  fill_msg(window_visitor, str_hash("channel-2"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_FALSE(ret);

  // fusioned
  make_msg_fusioned(window_visitor, str_hash("channel-1"));
  make_msg_fusioned(window_visitor, str_hash("channel-2"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2);
  EXPECT_TRUE(ret);
}

TEST(DataVisitorTest, four_channel_test) {
  auto channels = channel_vec;
  DataVisitor newest_visitor(std::move(channels), 1);
  channels = channel_vec;
  DataVisitor window_visitor(std::move(channels), 1,
                             FusionType::TIME_WINDOW);
  channels = channel_vec;
  DataVisitor callback_visitor(std::move(channels), 1);
  auto cb = callback;
  callback_visitor.RegisterCallback(std::move(cb));
  std::shared_ptr<RawMessage> msg0;
  std::shared_ptr<RawMessage> msg1;
  std::shared_ptr<RawMessage> msg2;
  std::shared_ptr<RawMessage> msg3;

  // empty
  bool ret;
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);
  ret =
      callback_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);
  // fill msg0, but msg1, msg2 and msg3's cache is still empty
  fill_msg0(newest_visitor);
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);
  fill_msg0(window_visitor);
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);

  // fill msg1, but msg2 and msg3's cache is still empty
  fill_msg(newest_visitor, str_hash("channel-1"));
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);
  fill_msg(window_visitor, str_hash("channel-1"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);

  // fill msg2, but msg3's cache is still empty
  fill_msg(newest_visitor, str_hash("channel-2"));
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);
  fill_msg(window_visitor, str_hash("channel-2"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);

  // fill msg3, but not fusioned
  fill_msg(newest_visitor, str_hash("channel-3"));
  ret = newest_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_TRUE(ret);
  fill_msg(window_visitor, str_hash("channel-3"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_FALSE(ret);

  // fusioned
  make_msg_fusioned(window_visitor, str_hash("channel-1"));
  make_msg_fusioned(window_visitor, str_hash("channel-2"));
  make_msg_fusioned(window_visitor, str_hash("channel-3"));
  ret = window_visitor.TryFetch<RawMessage, RawMessage>(msg0, msg1, msg2, msg3);
  EXPECT_TRUE(ret);
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
