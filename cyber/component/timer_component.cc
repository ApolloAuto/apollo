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

#include "cyber/component/timer_component.h"

#include "cyber/statistics/statistics.h"
#include "cyber/timer/timer.h"

namespace apollo {
namespace cyber {

TimerComponent::TimerComponent() {}

TimerComponent::~TimerComponent() {}

bool TimerComponent::Process() {
  if (is_shutdown_.load()) {
    return true;
  }
  return Proc();
}

bool TimerComponent::Initialize(const TimerComponentConfig& config) {
  if (!config.has_name() || !config.has_interval()) {
    AERROR << "Missing required field in config file.";
    return false;
  }
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);
  if (!Init()) {
    return false;
  }
  interval_ = config.interval();

  auto role_attr = std::make_shared<proto::RoleAttributes>();
  role_attr->set_node_name(config.name());
  role_attr->set_channel_name(statistics::TIMER_COMPONENT_CHAN_NAME);
  statistics::Statistics::Instance()->RegisterChanVar(*role_attr);

  std::shared_ptr<TimerComponent> self =
      std::dynamic_pointer_cast<TimerComponent>(shared_from_this());
  auto func = [self, role_attr]() {
    auto start_time = Time::Now().ToNanosecond();
    self->Process();
    auto end_time = Time::Now().ToNanosecond();
    // sampling proc latency in microsecond
    statistics::Statistics::Instance()->SamplingProcLatency<
                  uint64_t>(*role_attr, (end_time-start_time)/1000);
  };
  timer_.reset(new Timer(config.interval(), func, false));
  timer_->Start();
  return true;
}

void TimerComponent::Clear() { timer_.reset(); }

uint32_t TimerComponent::GetInterval() const { return interval_; }

}  // namespace cyber
}  // namespace apollo
