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

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/onboard/camera_process_subnode.h"
#include "modules/perception/obstacle/onboard/fusion_subnode.h"
#include "modules/perception/obstacle/onboard/async_fusion_subnode.h"
#include "modules/perception/obstacle/onboard/lane_post_processing_subnode.h"
#include "modules/perception/obstacle/onboard/lane_shared_data.h"
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/obstacle/onboard/lidar_process_subnode.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/fusion_shared_data.h"
#include "modules/perception/obstacle/onboard/radar_process_subnode.h"
#include "modules/perception/obstacle/onboard/visualization_subnode.h"
#include "modules/perception/obstacle/onboard/cipv_subnode.h"
#include "modules/perception/traffic_light/onboard/tl_preprocessor_subnode.h"
#include "modules/perception/traffic_light/onboard/tl_proc_subnode.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;

std::string Perception::Name() const { return "perception"; }

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
        << config_manager->WorkRoot();

  const std::string dag_config_path = apollo::common::util::GetAbsolutePath(
      FLAGS_work_root, FLAGS_dag_config_path);

  if (!dag_streaming_.Init(dag_config_path)) {
    AERROR << "failed to Init DAGStreaming. dag_config_path:"
           << dag_config_path;
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init DAGStreaming.");
  }
  callback_thread_num_ = 5;

  return Status::OK();
}

void Perception::RegistAllOnboardClass() {
  /// regist sharedata
  RegisterFactoryLidarObjectData();
  RegisterFactoryRadarObjectData();
  RegisterFactoryCameraObjectData();
  RegisterFactoryCameraSharedData();
  RegisterFactoryCIPVObjectData();
  RegisterFactoryLaneSharedData();
  RegisterFactoryFusionSharedData();
  traffic_light::RegisterFactoryTLPreprocessingData();

  /// regist subnode
  RegisterFactoryLidarProcessSubnode();
  RegisterFactoryRadarProcessSubnode();
  RegisterFactoryCameraProcessSubnode();
  RegisterFactoryCIPVSubnode();
  RegisterFactoryLanePostProcessingSubnode();
  RegisterFactoryAsyncFusionSubnode();
  RegisterFactoryFusionSubnode();
  RegisterFactoryMotionService();
  lowcostvisualizer::RegisterFactoryVisualizationSubnode();
  traffic_light::RegisterFactoryTLPreprocessorSubnode();
  traffic_light::RegisterFactoryTLProcSubnode();
}

Status Perception::Start() {
  dag_streaming_.Start();
  return Status::OK();
}

void Perception::Stop() {
  dag_streaming_.Stop();
  dag_streaming_.Join();
}

}  // namespace perception
}  // namespace apollo
