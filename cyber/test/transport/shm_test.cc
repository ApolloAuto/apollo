#include "gtest/gtest.h"
//#include <gmock/gmock.h>

#include "cyber/common/log.h"
#include "cyber/common/util.h"
#include "cyber/message/raw_message.h"
#include "cyber/transport/common/identity.h"
#include "cyber/transport/message/message_info.h"
#include "cyber/transport/shm/block.h"
#include "cyber/transport/shm/readable_info.h"
#include "cyber/transport/shm/segment.h"
#include "cyber/transport/shm/segment.h"
#include "cyber/transport/shm/shm_conf.h"
#include "cyber/transport/shm/state.h"
#include "transport_mocker.h"

namespace apollo {
namespace cyber {
namespace transport {

// shmget shmat shmctl shmdt
using cyber::message::RawMessage;

TEST(BlockTest, read_write_test) {
  Block block;

  // normal
  block.TryLockForWrite();
  uint8_t dest[1024];
  std::string message = "message";
  MessageInfo info;
  info.set_seq_num(1024);
  std::string message_info;
  info.SerializeTo(&message_info);
  block.Write(dest, message, message_info);
  block.ReleaseWriteLock();
  block.TryLockForRead();
  EXPECT_FALSE(block.Read(dest, nullptr, &message_info));
  EXPECT_FALSE(block.Read(dest, &message, nullptr));
  EXPECT_TRUE(block.Read(dest, &message, &message_info));
  EXPECT_EQ("message", message);
  info.DeserializeFrom(message_info);
  EXPECT_EQ(1024, info.seq_num());

  // read multi
  EXPECT_TRUE(block.TryLockForRead());
  EXPECT_TRUE(block.Read(dest, &message, &message_info));
  EXPECT_EQ("message", message);
  info.DeserializeFrom(message_info);
  block.ReleaseReadLock();
  block.ReleaseReadLock();

  // abnormal
  block.TryLockForWrite();
  EXPECT_FALSE(block.TryLockForWrite());
  EXPECT_FALSE(block.TryLockForRead());
  block.ReleaseWriteLock();
}

TEST(ReadableInfoTest, readable_info_test) {
  auto channel_id = common::Hash("channel");
  ReadableInfo empty_readable_info;
  EXPECT_EQ(0, empty_readable_info.host_id());
  EXPECT_EQ(0, empty_readable_info.block_index());
  EXPECT_EQ(0, empty_readable_info.channel_id());
  empty_readable_info.set_host_id(512);
  empty_readable_info.set_block_index(1024);
  empty_readable_info.set_channel_id(channel_id);
  std::string info;
  empty_readable_info.SerializeTo(&info);
  ReadableInfo readable_info(0, 0, channel_id);
  EXPECT_FALSE(readable_info.DeserializeFrom(""));
  EXPECT_TRUE(readable_info.DeserializeFrom(info));
  EXPECT_EQ(512, readable_info.host_id());
  EXPECT_EQ(1024, readable_info.block_index());
}

TEST(SegmentTest, segment_creation_test) {
  auto channel_id = common::Hash("channel");
  Segment read_segment(channel_id, READ_ONLY);
  Segment write_segment(channel_id, WRITE_ONLY);

  // destroy before init
  EXPECT_TRUE(read_segment.Destroy());
  EXPECT_TRUE(write_segment.Destroy());

  // segment should be created before read
  EXPECT_FALSE(read_segment.Init());
  EXPECT_TRUE(write_segment.Init());
  EXPECT_TRUE(write_segment.Init());
  EXPECT_TRUE(read_segment.Init());
  EXPECT_TRUE(read_segment.Init());

  read_segment.Destroy();
  write_segment.Destroy();
}

TEST(ShmConfTest, shm_conf_test) {
  ShmConf shm_conf1;
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_16K, shm_conf1.ceiling_msg_size());
  ShmConf shm_conf(ShmConf::MESSAGE_SIZE_16K - 1);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_16K, shm_conf.ceiling_msg_size());

  shm_conf.Update(ShmConf::MESSAGE_SIZE_128K - 1);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_128K, shm_conf.ceiling_msg_size());
  shm_conf.Update(ShmConf::MESSAGE_SIZE_128K);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_128K, shm_conf.ceiling_msg_size());

  shm_conf.Update(ShmConf::MESSAGE_SIZE_1M - 1);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_1M, shm_conf.ceiling_msg_size());
  shm_conf.Update(ShmConf::MESSAGE_SIZE_1M);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_1M, shm_conf.ceiling_msg_size());
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_1M + ShmConf::MESSAGE_INFO_SIZE,
            shm_conf.block_buf_size());
  EXPECT_EQ(64, shm_conf.block_num());
  AINFO << "Managed shm size of 1M: " << shm_conf.managed_shm_size();

  shm_conf.Update(ShmConf::MESSAGE_SIZE_8M - 1);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_8M, shm_conf.ceiling_msg_size());
  shm_conf.Update(ShmConf::MESSAGE_SIZE_8M);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_8M, shm_conf.ceiling_msg_size());

  shm_conf.Update(ShmConf::MESSAGE_SIZE_16M - 1);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_16M, shm_conf.ceiling_msg_size());
  shm_conf.Update(ShmConf::MESSAGE_SIZE_16M);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_16M, shm_conf.ceiling_msg_size());

  shm_conf.Update(ShmConf::MESSAGE_SIZE_MORE - 1);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_MORE, shm_conf.ceiling_msg_size());
  shm_conf.Update(ShmConf::MESSAGE_SIZE_MORE);
  EXPECT_EQ(ShmConf::MESSAGE_SIZE_MORE, shm_conf.ceiling_msg_size());

  State state(1024);
  state.IncreaseWroteNum();
  state.IncreaseWroteNum();
  EXPECT_EQ(2, state.wrote_num());
  state.ResetWroteNum();
  EXPECT_EQ(0, state.wrote_num());
}

TEST(SegmentTest, segment_read_write_test) {
  Identity sender_id;
  sender_id.set_data("sender");
  Identity spare_id;
  spare_id.set_data("spare");
  uint32_t block_index;
  std::string read_message;
  std::string writing_message;
  std::string message_info;
  MessageInfo info(sender_id, 0, spare_id);

  Segment read_segment(common::Hash("channel"), READ_ONLY);
  Segment write_segment(common::Hash("channel"), WRITE_ONLY);

  // destroy before init
  EXPECT_TRUE(read_segment.Destroy());
  EXPECT_TRUE(write_segment.Destroy());

  EXPECT_FALSE(read_segment.Read(12, &read_message, &message_info));
  for (int i = 0; i < write_segment.conf_.block_num() + 10; i++) {
    info.set_seq_num(i);
    writing_message = "message_" + std::to_string(i);
    info.SerializeTo(&message_info);
    EXPECT_TRUE(
        write_segment.Write(writing_message, message_info, &block_index));
    EXPECT_EQ(i % write_segment.conf_.block_num(), block_index);
    read_segment.Read(i % write_segment.conf_.block_num(), &read_message,
                      &message_info);
    EXPECT_TRUE(read_segment.Read(i % write_segment.conf_.block_num(),
                                  &read_message, &message_info));
    info.DeserializeFrom(message_info);
    EXPECT_EQ(i, info.seq_num());
  }

  EXPECT_TRUE(read_segment.Read(0, &read_message, &message_info));
  EXPECT_FALSE(read_segment.Read(0, nullptr, &message_info));
  EXPECT_FALSE(read_segment.Read(0, &read_message, nullptr));

  // remap
  write_segment.state_->set_need_remap(true);
  EXPECT_TRUE(write_segment.Write(writing_message, message_info, &block_index));
  write_segment.state_->set_need_remap(false);

  // recreate
  std::string large(1024 * 1024, 'a');
  EXPECT_TRUE(write_segment.Write(large, message_info, &block_index));
  EXPECT_TRUE(read_segment.Read(0, &read_message, &message_info));
  EXPECT_TRUE(read_segment.Read(0, &read_message, &message_info));

  // clear
  write_segment.Destroy();
  read_segment.Destroy();
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
