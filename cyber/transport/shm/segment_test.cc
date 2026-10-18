/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cyber/transport/shm/segment.h"

#include <cstdint>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace transport {
namespace {

constexpr uint64_t kSmallMessageSize = 16 * 1024;
constexpr uint64_t kLargeMessageSize = 32 * 1024 * 1024;

// Replace only the backing storage so that Segment's read and remap paths can
// be exercised without timing-dependent shared-memory notifications.
class InMemorySegment : public Segment {
 public:
  explicit InMemorySegment(uint64_t msg_size = kSmallMessageSize)
      : Segment(0), next_msg_size_(msg_size) {
    OpenOnly();
  }

  void RequestRemap(uint64_t msg_size, bool open_success = true) {
    // Keep the old configuration until AcquireBlockToRead calls Remap(), as
    // happens when another process recreates the shared-memory segment.
    next_msg_size_ = msg_size;
    open_success_ = open_success;
    state_->set_need_remap(true);
  }

  uint32_t block_num() { return conf_.block_num(); }

 protected:
  void Reset() override {
    init_ = false;
    state_ = nullptr;
    blocks_ = nullptr;
    block_buf_addrs_.clear();
    block_storage_.reset();
    state_storage_.reset();
    buffers_.clear();
  }

  bool Remove() override { return true; }

  bool OpenOnly() override {
    if (!open_success_) {
      return false;
    }
    conf_.Update(next_msg_size_);
    state_storage_.reset(new State(conf_.ceiling_msg_size()));
    block_storage_.reset(new Block[conf_.block_num()]);
    // Reading a block only needs a buffer address, not a full message payload.
    buffers_.resize(conf_.block_num());
    state_ = state_storage_.get();
    blocks_ = block_storage_.get();
    for (uint32_t i = 0; i < conf_.block_num(); ++i) {
      block_buf_addrs_[i] = &buffers_[i];
    }
    init_ = true;
    return true;
  }

  bool OpenOrCreate() override { return OpenOnly(); }

 private:
  uint64_t next_msg_size_;
  bool open_success_ = true;
  std::unique_ptr<State> state_storage_;
  std::unique_ptr<Block[]> block_storage_;
  std::vector<uint8_t> buffers_;
};

}  // namespace

class SegmentRemapTest : public ::testing::TestWithParam<bool> {};

TEST_P(SegmentRemapTest, reject_stale_index) {
  InMemorySegment segment;
  ShmConf next_conf(kLargeMessageSize);
  ASSERT_LT(next_conf.block_num(), segment.block_num());

  // A queued notification can refer to either the first invalid block or the
  // last block of the old mapping when a larger message recreates the segment.
  ReadableBlock readable_block;
  readable_block.index =
      GetParam() ? segment.block_num() - 1 : next_conf.block_num();
  segment.RequestRemap(kLargeMessageSize);

  EXPECT_FALSE(segment.AcquireBlockToRead(&readable_block));
  EXPECT_EQ(segment.block_num(), next_conf.block_num());
  EXPECT_EQ(readable_block.block, nullptr);
  EXPECT_EQ(readable_block.buf, nullptr);
}

INSTANTIATE_TEST_SUITE_P(ShrinkingMapping, SegmentRemapTest, ::testing::Bool());

TEST(SegmentTest, read_after_remap) {
  InMemorySegment segment;
  ShmConf next_conf(kLargeMessageSize);
  ReadableBlock readable_block;
  readable_block.index = next_conf.block_num() - 1;
  segment.RequestRemap(kLargeMessageSize);

  ASSERT_TRUE(segment.AcquireBlockToRead(&readable_block));
  EXPECT_EQ(segment.block_num(), next_conf.block_num());
  EXPECT_NE(readable_block.block, nullptr);
  EXPECT_NE(readable_block.buf, nullptr);
  segment.ReleaseReadBlock(readable_block);
}

TEST(SegmentTest, read_index_valid_in_new_mapping) {
  InMemorySegment segment(kLargeMessageSize);
  ShmConf next_conf(kSmallMessageSize);
  ReadableBlock readable_block;
  readable_block.index = next_conf.block_num() - 1;
  ASSERT_GE(readable_block.index, segment.block_num());
  // An index outside the old mapping can be valid in the replacement mapping.
  segment.RequestRemap(kSmallMessageSize);

  ASSERT_TRUE(segment.AcquireBlockToRead(&readable_block));
  EXPECT_EQ(segment.block_num(), next_conf.block_num());
  EXPECT_NE(readable_block.block, nullptr);
  EXPECT_NE(readable_block.buf, nullptr);
  segment.ReleaseReadBlock(readable_block);
}

TEST(SegmentTest, failed_remap) {
  InMemorySegment segment;
  ReadableBlock readable_block;
  // Reset discards the old blocks even when opening the new segment fails.
  segment.RequestRemap(kLargeMessageSize, /*open_success=*/false);

  EXPECT_FALSE(segment.AcquireBlockToRead(&readable_block));
  EXPECT_EQ(readable_block.block, nullptr);
  EXPECT_EQ(readable_block.buf, nullptr);
}

TEST(SegmentTest, read_without_remap) {
  InMemorySegment segment;
  ReadableBlock readable_block;
  readable_block.index = segment.block_num() - 1;

  ASSERT_TRUE(segment.AcquireBlockToRead(&readable_block));
  EXPECT_NE(readable_block.block, nullptr);
  EXPECT_NE(readable_block.buf, nullptr);
  segment.ReleaseReadBlock(readable_block);
}

TEST(SegmentTest, invalid_read_without_remap) {
  InMemorySegment segment;
  ReadableBlock readable_block;
  readable_block.index = segment.block_num();

  EXPECT_FALSE(segment.AcquireBlockToRead(&readable_block));
  EXPECT_FALSE(segment.AcquireBlockToRead(nullptr));
  EXPECT_EQ(readable_block.block, nullptr);
  EXPECT_EQ(readable_block.buf, nullptr);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
