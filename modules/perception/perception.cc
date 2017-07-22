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

#include <sensor_msgs/PointCloud2.h>

#include "modules/perception/perception.h"
//#include "modules/perception/obstacle/lidar/onboard/lidar_predetection_data.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;

std::string Perception::Name() const { return "perception"; }

Status Perception::Init() {
  
  AdapterManager::Init(FLAGS_adapter_config_path);
//  preprocessor_.Init(FLAGS_perception_flag);

  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::SetPointCloudCallback(&Perception::OnPointCloud, this);
  return Status::OK();
}

void Perception::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  AINFO << "get point cloud callback";
  //auto predetection_data = std::make_shared<LidarPredectionData>();

  //if(preprocessor_.Proc(message, &predetection_data) != Status.OK()) {
  //  return Status(ERROR_CODE::PERCEPTION_ERROR, "preprocessing failure in perception");
  //}

  //adding other logic here
}

Status Perception::Start() {
  return Status::OK();
}

void Perception::Stop() {}

}  // namespace perception
}  // namespace apollo
