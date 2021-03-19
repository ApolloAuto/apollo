/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/canbus/can_client/fake/fake_can_client.h"

#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

class FakeCanClientTest : public ::testing::Test {
 public:
  static const int32_t FRAME_LEN = 10;

  virtual void SetUp() {
    send_time_ = 0;
    recv_time_ = 0;
    send_succ_count_ = 0;
    recv_succ_count_ = 0;
    send_err_count_ = 0;
    recv_err_count_ = 0;
    param_.set_brand(CANCardParameter::ESD_CAN);
    param_.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
    send_client_ = std::unique_ptr<FakeCanClient>(new FakeCanClient());
    send_client_->Init(param_);
    send_client_->Start();
    recv_client_ = std::unique_ptr<FakeCanClient>(new FakeCanClient());
    recv_client_->Init(param_);
    recv_client_->Start();
  }

 protected:
  std::unique_ptr<FakeCanClient> send_client_;
  std::unique_ptr<FakeCanClient> recv_client_;

  int64_t send_time_ = 0;
  int64_t recv_time_ = 0;
  int32_t send_succ_count_ = 0;
  int32_t recv_succ_count_ = 0;
  int32_t send_err_count_ = 0;
  int32_t recv_err_count_ = 0;
  std::stringstream recv_ss_;
  CANCardParameter param_;
};

TEST_F(FakeCanClientTest, SendMessage) {
  std::vector<CanFrame> frames;
  frames.resize(FRAME_LEN);
  for (int32_t i = 0; i < FRAME_LEN; ++i) {
    frames[i].id = 1 & 0x3FF;
    frames[i].len = 8;
    frames[i].data[7] = 1 % 256;
    for (uint8_t j = 0; j < 7; ++j) {
      frames[i].data[j] = j;
    }
  }

  int32_t frame_num = FRAME_LEN;
  auto ret = send_client_->Send(frames, &frame_num);
  EXPECT_EQ(ret, ErrorCode::OK);
  EXPECT_EQ(send_client_->GetErrorString(0), "");
  send_client_->Stop();
}

TEST_F(FakeCanClientTest, ReceiveMessage) {
  std::vector<CanFrame> buf;
  int32_t frame_num = FRAME_LEN;

  auto ret = recv_client_->Receive(&buf, &frame_num);
  EXPECT_EQ(ret, ErrorCode::OK);
  recv_client_->Stop();
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
