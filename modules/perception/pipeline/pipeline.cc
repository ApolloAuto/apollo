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

#include "modules/perception/pipeline/pipeline.h"

#include "modules/common/util/map_util.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/detection.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.h"
#include "modules/perception/camera/lib/traffic_light/tracker/semantic_decision.h"
#include "modules/perception/lidar/lib/classifier/fused_classifier/fused_classifier.h"
#include "modules/perception/lidar/lib/detector/point_pillars_detection/point_pillars_detection.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_engine.h"


namespace apollo {
namespace perception {
namespace pipeline {


bool Pipeline::Initialize(const PipelineConfig& pipeline_config) {
  ACHECK(!pipeline_config.stage_type().empty());

  Clear();

  for (const auto& stage_config : pipeline_config.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = stage_config;
  }

  for (int i = 0; i < pipeline_config.stage_config_size(); ++i) {
    auto stage_type = pipeline_config.stage_type(i);
    if (!apollo::common::util::ContainsKey(stage_config_map_, stage_type)) {
      AERROR << "Stage type : " << StageType_Name(stage_type)
             << " has no config";
      return false;
    }

    std::unique_ptr<Stage> stage_ptr = CreateStage(stage_type);

    if (stage_ptr == nullptr) {
      AERROR << "Create stage type : " << StageType_Name(stage_type)
             << " failed!";
      return false;
    }

    stage_ptrs_.push_back(std::move(stage_ptr));
  }

  name_ = PipelineType_Name(pipeline_config.pipeline_type());
  pipeline_config_.CopyFrom(pipeline_config);

  return true;
}

bool Pipeline::InnerProcess(DataFrame* frame) {
  for (const auto& stage_ptr : stage_ptrs_) {
    if (stage_ptr->IsEnabled()) {
      bool res = stage_ptr->Process(frame);
      if (!res) {
        AERROR << "Pipeline: " << name_
               << " Stage : " << stage_ptr->Name() << " failed!";
        return false;
      }
    }
  }
  return true;
}

std::unique_ptr<Stage> Pipeline::CreateStage(const StageType& stage_type) {
  std::unique_ptr<Stage> stage_ptr;
  switch (stage_type) {
    case StageType::POINTCLOUD_PREPROCESSOR:
      stage_ptr.reset(new lidar::PointCloudPreprocessor());
      break;
    case StageType::MAP_MANAGER:
      stage_ptr.reset(new lidar::MapManager());
      break;
    case StageType::POINT_PILLARS_DETECTION:
      stage_ptr.reset(new lidar::PointPillarsDetection());
      break;
    case StageType::OBJECT_BUILDER:
      stage_ptr.reset(new lidar::ObjectBuilder());
      break;
    case StageType::OBJECT_FILTER_BANK:
      stage_ptr.reset(new lidar::ObjectFilterBank());
      break;
    case StageType::MLF_ENGINE:
      stage_ptr.reset(new lidar::MlfEngine());
      break;
    case StageType::FUSED_CLASSIFIER:
      stage_ptr.reset(new lidar::FusedClassifier());
      break;
    case StageType::TRAFFIC_LIGHT_DETECTION:
      stage_ptr.reset(new camera::TrafficLightDetection());
      break;
    case StageType::TRAFFIC_LIGHT_RECOGNITION:
      stage_ptr.reset(new camera::TrafficLightRecognition());
      break;
    case StageType::SEMANTIC_REVISER:
      stage_ptr.reset(new camera::SemanticReviser());
      break;
    default:
      return nullptr;
  }

  if (stage_ptr != nullptr)
    stage_ptr->Init(stage_config_map_[stage_type]);

  return stage_ptr;
}

void Pipeline::Clear() {
  stage_ptrs_.clear();
  stage_config_map_.clear();
}

} // namespace pipeline
} // namespace perception
} // namespace apollo
