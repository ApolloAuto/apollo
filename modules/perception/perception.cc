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
#include "modules/common/log.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/base/file_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/onboard/lidar_process_subnode.h"
#include "modules/perception/obstacle/onboard/radar_process_subnode.h"
#include "modules/perception/obstacle/onboard/fusion_subnode.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/traffic_light/onboard/preprocessor_subnode.h"
#include "modules/perception/traffic_light/onboard/proc_subnode.h"
#include "modules/perception/onboard/dag_streaming.h"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

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

  RegistAllOnboardClass();
  /// init config manager
  ConfigManager* config_manager = ConfigManager::instance();
  if (!config_manager->Init()) {
      AERROR << "failed to Init ConfigManager";
      return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init ConfigManager.");
  }
  AINFO << "Init config manager successfully, work_root: "
        << config_manager->work_root();

  const std::string dag_config_path =
      FileUtil::GetAbsolutePath(FLAGS_work_root, FLAGS_dag_config_path);

  DAGStreaming dag_streaming;
    if (!dag_streaming.Init(dag_config_path)) {
    AERROR << "failed to Init DAGStreaming. dag_config_path:"
           << dag_config_path;
        return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init DAGStreaming.");
  }

  dag_streaming.Start();
  return Status::OK();
}

void Perception::RegistAllOnboardClass() {
  /// regist subnode
  RegisterFactoryLidarProcessSubnode();
  RegisterFactoryRadarProcessSubnode();
  RegisterFactoryFusionSubnode();
  traffic_light::RegisterFactoryTLPreprocessorSubnode();
  traffic_light::RegisterFactoryTLProcSubnode();

  /// regist sharedata
  RegisterFactoryLidarObjectData();
  RegisterFactoryRadarObjectData();
  traffic_light::RegisterFactoryTLPreprocessingData();
  traffic_light::RegisterFactoryTLProcData();
}

Status Perception::Start() {
  return Status::OK();
}

void Perception::Stop() {}

}  // namespace perception
}  // namespace apollo
