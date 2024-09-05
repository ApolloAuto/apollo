/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "cyber/transport/shm/shm_conf.h"
#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

ShmConf::ShmConf() { Update(MESSAGE_SIZE_16K); }

ShmConf::ShmConf(const uint64_t& real_msg_size) { Update(real_msg_size); }

ShmConf::~ShmConf() {}

void ShmConf::Update(const uint64_t& real_msg_size) {
  ceiling_msg_size_ = GetCeilingMessageSize(real_msg_size);
  block_buf_size_ = GetBlockBufSize(ceiling_msg_size_);
  block_num_ = GetBlockNum(ceiling_msg_size_);
  managed_shm_size_ = EXTRA_SIZE + STATE_SIZE + \
                      (BLOCK_SIZE + block_buf_size_) * block_num_ + \
                      (BLOCK_SIZE + ARENA_MESSAGE_SIZE) * ARENA_BLOCK_NUM;
}

const uint64_t ShmConf::EXTRA_SIZE = 1024 * 4;
const uint64_t ShmConf::STATE_SIZE = 1024;
const uint64_t ShmConf::BLOCK_SIZE = 1024;
const uint64_t ShmConf::MESSAGE_INFO_SIZE = 1024;

const uint32_t ShmConf::ARENA_BLOCK_NUM = 512;
const uint64_t ShmConf::ARENA_MESSAGE_SIZE = 1024;

const uint32_t ShmConf::BLOCK_NUM_16K = 512;
const uint64_t ShmConf::MESSAGE_SIZE_16K = 1024 * 16;

const uint32_t ShmConf::BLOCK_NUM_128K = 128;
const uint64_t ShmConf::MESSAGE_SIZE_128K = 1024 * 128;

const uint32_t ShmConf::BLOCK_NUM_1M = 64;
const uint64_t ShmConf::MESSAGE_SIZE_1M = 1024 * 1024;

const uint32_t ShmConf::BLOCK_NUM_8M = 32;
const uint64_t ShmConf::MESSAGE_SIZE_8M = 1024 * 1024 * 8;

const uint32_t ShmConf::BLOCK_NUM_16M = 16;
const uint64_t ShmConf::MESSAGE_SIZE_16M = 1024 * 1024 * 16;

const uint32_t ShmConf::BLOCK_NUM_MORE = 8;
const uint64_t ShmConf::MESSAGE_SIZE_MORE = 1024 * 1024 * 32;

uint64_t ShmConf::GetCeilingMessageSize(const uint64_t& real_msg_size) {
  uint64_t ceiling_msg_size = MESSAGE_SIZE_16K;
  if (real_msg_size <= MESSAGE_SIZE_16K) {
    ceiling_msg_size = MESSAGE_SIZE_16K;
  } else if (real_msg_size <= MESSAGE_SIZE_128K) {
    ceiling_msg_size = MESSAGE_SIZE_128K;
  } else if (real_msg_size <= MESSAGE_SIZE_1M) {
    ceiling_msg_size = MESSAGE_SIZE_1M;
  } else if (real_msg_size <= MESSAGE_SIZE_8M) {
    ceiling_msg_size = MESSAGE_SIZE_8M;
  } else if (real_msg_size <= MESSAGE_SIZE_16M) {
    ceiling_msg_size = MESSAGE_SIZE_16M;
  } else {
    ceiling_msg_size = MESSAGE_SIZE_MORE;
  }
  return ceiling_msg_size;
}

uint64_t ShmConf::GetBlockBufSize(const uint64_t& ceiling_msg_size) {
  return ceiling_msg_size + MESSAGE_INFO_SIZE;
}

uint32_t ShmConf::GetBlockNum(const uint64_t& ceiling_msg_size) {
  uint32_t num = 0;
  switch (ceiling_msg_size) {
    case MESSAGE_SIZE_16K:
      num = BLOCK_NUM_16K;
      break;
    case MESSAGE_SIZE_128K:
      num = BLOCK_NUM_128K;
      break;
    case MESSAGE_SIZE_1M:
      num = BLOCK_NUM_1M;
      break;
    case MESSAGE_SIZE_8M:
      num = BLOCK_NUM_8M;
      break;
    case MESSAGE_SIZE_16M:
      num = BLOCK_NUM_16M;
      break;
    case MESSAGE_SIZE_MORE:
      num = BLOCK_NUM_MORE;
      break;
    default:
      AERROR << "unknown ceiling_msg_size[" << ceiling_msg_size << "]";
      break;
  }
  return num;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
