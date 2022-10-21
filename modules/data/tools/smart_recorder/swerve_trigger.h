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

#include <deque>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/data/tools/smart_recorder/proto/smart_recorder_triggers.pb.h"
#include "modules/data/tools/smart_recorder/trigger_base.h"

namespace apollo {
namespace data {

/**
 * @class SwerveTrigger
 * @brief Swerve trigger that fires when swerve is engaged
 */
class SwerveTrigger : public TriggerBase {
 public:
  SwerveTrigger();

  void Pull(const cyber::record::RecordMessage& msg) override;
  bool ShouldRestore(const cyber::record::RecordMessage& msg) const override {
    return false;
  };

  virtual ~SwerveTrigger() = default;

 private:
  bool IsSwerve() const;
  bool IsNoisy(const float steer) const;
  void EnqueueMessage(const float steer);

 private:
  const size_t queue_size_ = 10;
  const float max_delta_ = 10.0f;
  const float noisy_diff_ = 20.0f;
  std::deque<float> history_steer_queue_;
  std::deque<float> current_steer_queue_;
  float history_total_;
  float current_total_;
};

}  // namespace data
}  // namespace apollo
