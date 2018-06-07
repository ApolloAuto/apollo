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

#include "modules/perception/obstacle/onboard/ultrasonic_obstacle_subnode.h"

#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

void UltrasonicObstacleSubnode::OnUltrasonic(
    const apollo::canbus::Chassis& message) {
  ++seq_num_;
  PERF_BLOCK_START();
  std::shared_ptr<SensorObjects> sensor_objects(new SensorObjects);
  double timestamp = message.header().timestamp_sec();
  sensor_objects->timestamp = timestamp;
  sensor_objects->sensor_type = SensorType::ULTRASONIC;
  sensor_objects->sensor_id = device_id_;
  sensor_objects->seq_num = seq_num_;

  // TODO(all) transform message to object and insert into sensor_objects

  if (!PublishDataAndEvent(timestamp, sensor_objects)) {
    AERROR << "Failed to publish data.";
    sensor_objects->error_code = apollo::common::PERCEPTION_ERROR_PROCESS;
    return;
  }
  ADEBUG << "ultrasonic object size: " << sensor_objects->objects.size();
  PERF_BLOCK_END("ultrasonic_detect");
}

bool UltrasonicObstacleSubnode::PublishDataAndEvent(
    const double timestamp, const SharedDataPtr<SensorObjects>& data) {
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key)) {
    AERROR << "Failed to produce shared key. time: "
           << GLOG_TIMESTAMP(timestamp) << ", device_id: " << device_id_;
    return false;
  }

  // TODO(all) processing_data_->Add(key, data);

  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta& event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }

  return true;
}

}  // namespace perception
}  // namespace apollo
