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

#include "modules/perception/perception.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/onboard/lidar_process.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;

std::string Perception::Name() const {
  return "perception";
}

Status Perception::Init() {
  AdapterManager::Init(FLAGS_perception_adapter_config_filename);

  lidar_process_.reset(new LidarProcess());
  if (lidar_process_ != nullptr && !lidar_process_->Init()) {
    AERROR << "failed to init lidar_process.";
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to init lidar_process.");
  }

  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback(&Perception::RunOnce, this);
  return Status::OK();
}

void Perception::RunOnce(const sensor_msgs::PointCloud2& message) {
  ADEBUG << "get point cloud callback";

  if (lidar_process_ != nullptr && lidar_process_->IsInit()) {
    lidar_process_->Process(message);

    /// public obstacle message
    PerceptionObstacles obstacles;
    if (lidar_process_->GeneratePbMsg(&obstacles)) {
      AdapterManager::PublishPerceptionObstacles(obstacles);
    }
  }
}

Status Perception::Start() {
  return Status::OK();
}

void Perception::Stop() {}

}  // namespace perception
}  // namespace apollo
