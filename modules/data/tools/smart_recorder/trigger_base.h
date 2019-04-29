/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "cyber/record/record_message.h"
#include "modules/data/tools/smart_recorder/proto/smart_recorder_triggers.pb.h"

namespace apollo {
namespace data {

using cyber::record::RecordMessage;

/**
 * @class TriggerBase
 * @brief Base class of triggers that defines interfaces
 */
class TriggerBase {
 public:
  TriggerBase() = default;

  // Init the trigger using configuration
  virtual bool Init(const SmartRecordTrigger& trigger_conf);

  // Decide if the current trigger needs to be caught by input message,
  // and if yes record the time interval
  virtual void Pull(const RecordMessage& msg) = 0;

  // Decide if the current message needs to be restored
  virtual bool ShouldRestore(const RecordMessage& msg) const = 0;

  // Getters
  const std::unordered_map<std::string, std::string>& GetChannelTypes() const {
    return channels_types_map_;
  }
  const std::string GetTriggerName() const { return trigger_name_; }

  void InsertChannelType(const std::string& channel, const std::string& type) {
    channels_types_map_.insert({channel, type});
  }

  virtual ~TriggerBase() = default;

 protected:
  void TriggerIt(const uint64_t msg_time) const;

  std::string trigger_name_;
  std::unique_ptr<Trigger> trigger_obj_ = nullptr;

 private:
  void LockTrigger(const SmartRecordTrigger& trigger_conf);

  std::unordered_map<std::string, std::string> channels_types_map_;
};

}  // namespace data
}  // namespace apollo
