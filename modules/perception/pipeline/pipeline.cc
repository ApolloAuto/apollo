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


#include "modules/perception/lidar/lib/detector/point_pillars_detection/point_pillars_detection.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"
#include "modules/perception/pipeline/proto/traffic_light_config.pb.h"
#include "modules/common/util/map_util.h"

namespace apollo {
namespace perception {
namespace pipeline {


bool Pipeline::Initialize(const PipelineConfig& pipeline_config) {
  ACHECK(pipeline_config.stage_type().empty());

  Clear();

  for (const auto& stage_config : pipeline_config.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = &stage_config;
  }

  for (int i = 0; i < pipeline_config.stage_config_size(); ++i) {
    auto stage_type = pipeline_config.stage_type(i);
    if (!common::util::ContainsKey(stage_config_map_, stage_type)) {
      AERROR << "Stage type : " << StageType_Name(stage_type)
             << " has no config";
      return false;
    }

    std::unique_ptr<Stage> stage_ptr = CreateStage(stage_type);

    if (stage_ptr == nullptr) {
      AERROR << "Create stage type : " << StageType_Name(stage_type)
             << " failed!";
    } else {
      stage_ptrs_.push_back(stage_ptr);
    }
  }

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
      stage_ptr.reset(new PointCloudPreprocessor());
      break;
    case StageType::MAP_MANAGER:
      stage_ptr.reset(new MapManager());
      break;
    case StageType::POINT_PILLARS_DETECTION:
      stage_ptr.reset(new PointPillarsDetection());
      break;
    case StageType::OBJECT_BUILDER:
      stage_ptr.reset(new ObjectBuilder());
      break;
    case StageType::OBJECT_FILTER_BANK:
      stage_ptr.reset(new ObjectFilterBank());
      break;
    case StageType::TRAFFIC_LIGHT_DETECTION:
      stage_ptr.reset(new TrafficLightDetection());
      break;
    case StageType::TRAFFIC_LIGHT_RECOGNITION:
      stage_ptr.reset(new TrafficLightRecognition());
      break;
    case StageType::SEMANTIC_REVISER:
      stage_ptr.reset(new SemanticReviser());
      break;
    default:
      return nullptr;
  }

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
