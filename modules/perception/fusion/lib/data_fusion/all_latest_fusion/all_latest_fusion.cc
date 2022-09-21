/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/fusion/lib/data_fusion/all_latest_fusion/all_latest_fusion.h"


#include "modules/common/util/string_util.h"
#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/pipeline/data_frame.h"


namespace apollo {
namespace perception {
namespace fusion {


bool AllLatestFusion::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  all_latest_fusion_config_ = stage_config.all_latest_fusion_config();

  main_sensor_ = all_latest_fusion_config_.main_sensor();
  use_lidar_ = all_latest_fusion_config_.use_lidar();
  use_radar_ = all_latest_fusion_config_.use_radar();
  use_camera_ = all_latest_fusion_config_.use_camera();
  return true;
}

bool AllLatestFusion::Process(DataFrame* data_frame) {
  if (data_frame == nullptr) {
    return false;
  }

  FusionFrame* fusion_frame = data_frame->fusion_frame;
  if (fusion_frame == nullptr)
    return false;

  base::FrameConstPtr sensor_frame = fusion_frame->frame;
  if (sensor_frame == nullptr)
    return false;

  SensorDataManager* sensor_data_manager = SensorDataManager::Instance();
  // 1. save frame data
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    if (!use_lidar_ && sensor_data_manager->IsLidar(sensor_frame)) {
      return true;
    }
    if (!use_radar_ && sensor_data_manager->IsRadar(sensor_frame)) {
      return true;
    }
    if (!use_camera_ && sensor_data_manager->IsCamera(sensor_frame)) {
      return true;
    }
    AINFO << "add sensor measurement: " << sensor_frame->sensor_info.name
          << ", obj_cnt : " << sensor_frame->objects.size() << ", "
          << FORMAT_TIMESTAMP(sensor_frame->timestamp);

    sensor_data_manager->AddSensorMeasurements(sensor_frame);

    bool is_publish_sensor = IsPublishSensor(sensor_frame);
    if (!is_publish_sensor) {
      return true;
    }
  }

  // 2. query related sensor_frames for fusion
  std::lock_guard<std::mutex> fuse_lock(fuse_mutex_);
  double fusion_time = sensor_frame->timestamp;
  std::vector<SensorFramePtr>* frames = &fusion_frame->sensor_frames;
  sensor_data_manager->GetLatestFrames(fusion_time, frames);
  AINFO << "Get " << frames->size() << " related frames for fusion";

  return true;
}

bool AllLatestFusion::IsPublishSensor(
    const base::FrameConstPtr& sensor_frame) const {
  return main_sensor_ == sensor_frame->sensor_info.name;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
