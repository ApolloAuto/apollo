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

#ifndef ONBOARD_MODULES_CHECKIN_BUILD_CYBERTRON_WHISPER_COMMON_WHISPER_H_
#define ONBOARD_MODULES_CHECKIN_BUILD_CYBERTRON_WHISPER_COMMON_WHISPER_H_

#include <string>
#include <cstdlib>
#include "data_pool.h"
#include "cybertron/cybertron.h"
#include "cybertron/proto/cybertron_whisper.pb.h"
#include "cybertron/proto/header.pb.h"

class WhisperCommon {
 public:
  typedef apollo::cybertron::proto::CybertronWhisper ChannelType;
  typedef std::shared_ptr<ChannelType> ChannelPtr;
  typedef std::shared_ptr<const ChannelType> ConstChannelPtr;

  WhisperCommon() = delete;

  WhisperCommon(const std::string& me, const std::string& channel_name,
                const std::string& frame_id,
                const std::shared_ptr<DataPool>& data_pool);
  ~WhisperCommon();

  ChannelPtr next();
  std::string show(const ConstChannelPtr& whisper);
  static int32_t sleep_for(int32_t value, int32_t interval, int32_t step);

 private:
  std::string _me;
  std::string _channel_name;
  std::string _frame_id;
  int32_t _seq;
  int32_t _position;
  std::shared_ptr<DataPool> _data_pool;
  char* _allocating;  // so dont need to do syncrize
  uint64_t _last_receive_time;
  std::mutex _mutex;
};

#endif

