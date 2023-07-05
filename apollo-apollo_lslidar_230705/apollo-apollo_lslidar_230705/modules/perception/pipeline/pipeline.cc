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

#include "cyber/time/clock.h"
#include "modules/common/util/map_util.h"
#include "modules/perception/camera/lib/obstacle/camera_detection_postprocessor/camera_detection_postprocessor.h"
#include "modules/perception/camera/lib/obstacle/detector/bev_detection/bev_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/detector/caddn/caddn_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/detector/smoke/smoke_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/postprocessor/location_refiner/location_refiner_obstacle_postprocessor.h"
#include "modules/perception/camera/lib/obstacle/preprocessor/camera_detection_preprocessor.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/omt_obstacle_tracker.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt2/omt_bev_tracker.h"
#include "modules/perception/camera/lib/obstacle/transformer/multicue/multicue_obstacle_transformer.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/detection.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.h"
#include "modules/perception/camera/lib/traffic_light/tracker/semantic_decision.h"
#include "modules/perception/fusion/lib/data_fusion/all_latest_fusion/all_latest_fusion.h"
#include "modules/perception/fusion/lib/fusion_system/probabilistic_fusion/probabilistic_fusion.h"
#include "modules/perception/fusion/lib/gatekeeper/collect_fused_object.h"
#include "modules/perception/lidar/lib/classifier/fused_classifier/fused_classifier.h"
#include "modules/perception/lidar/lib/detector/center_point_detection/center_point_detection.h"
#include "modules/perception/lidar/lib/detector/cnn_segmentation/cnn_segmentation.h"
#include "modules/perception/lidar/lib/detector/mask_pillars_detection/mask_pillars_detection.h"
#include "modules/perception/lidar/lib/detector/ncut_segmentation/ncut_segmentation.h"
#include "modules/perception/lidar/lib/detector/point_pillars_detection/point_pillars_detection.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"
#include "modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_detection_postprocessor.h"
#include "modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_detection_preprocessor.h"
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_engine.h"
#include "modules/perception/pipeline/plugin_factory.h"

namespace apollo {
namespace perception {
namespace pipeline {

bool Pipeline::Initialize(const PipelineConfig& pipeline_config) {
  ACHECK(!pipeline_config.stage_type().empty());

  Clear();

  for (const auto& stage_config : pipeline_config.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = stage_config;
  }

  // Register Plugins, Must be initialized before 'CreateStage'!!!
  PluginFactory::Init();

  for (int i = 0; i < pipeline_config.stage_config_size(); ++i) {
    auto stage_type = pipeline_config.stage_type(i);
    if (!apollo::common::util::ContainsKey(stage_config_map_, stage_type)) {
      AERROR << "Stage type : " << StageType_Name(stage_type)
             << " has no config";
      return false;
    }

    if (!CheckRepeatedStage(StageType_Name(stage_type))) {
      std::shared_ptr<Stage> stage_ptr = CreateStage(stage_type);

      if (stage_ptr == nullptr) {
        AERROR << "Create stage type : " << StageType_Name(stage_type)
               << " failed!";
        return false;
      }

      AINFO << "Create stage type : " << StageType_Name(stage_type)
            << " success!";
      stage_ptrs_.push_back(std::move(stage_ptr));
    }
  }

  name_ = PipelineType_Name(pipeline_config.pipeline_type());
  pipeline_config_.CopyFrom(pipeline_config);

  return true;
}

bool Pipeline::CheckRepeatedStage(const std::string& stage_name) {
  bool res = false;
  for (auto created_state_ptr : stage_ptrs_) {
    if (StageType_Name(created_state_ptr->stage_config_.stage_type()) ==
        stage_name) {
      AINFO << stage_name << " already created";
      stage_ptrs_.push_back(std::move(created_state_ptr));
      res = true;
      break;
    }
  }
  return res;
}

bool Pipeline::InnerProcess(DataFrame* frame) {
  for (const auto& stage_ptr : stage_ptrs_) {
    if (stage_ptr->IsEnabled()) {
      double start_time = apollo::cyber::Clock::NowInSeconds();
      bool res = stage_ptr->Process(frame);
      AINFO << "Stage: " << stage_ptr->Name()
            << " Cost: " << apollo::cyber::Clock::NowInSeconds() - start_time;
      if (!res) {
        AERROR << "Pipeline: " << name_ << " Stage : " << stage_ptr->Name()
               << " failed!";
        return false;
      }
    } else {
      AINFO << "Pipeline: " << name_ << " Stage : " << stage_ptr->Name()
            << " disabled!";
    }
  }
  return true;
}

std::shared_ptr<Stage> Pipeline::CreateStage(const StageType& stage_type) {
  std::shared_ptr<Stage> stage_ptr;
  switch (stage_type) {
    case StageType::POINTCLOUD_PREPROCESSOR:
      stage_ptr.reset(new lidar::PointCloudPreprocessor());
      break;
    case StageType::POINTCLOUD_DETECTION_PREPROCESSOR:
      stage_ptr.reset(new lidar::PointcloudDetectionPreprocessor());
      break;
    case StageType::POINTCLOUD_DETECTION_POSTPROCESSOR:
      stage_ptr.reset(new lidar::PointcloudDetectionPostprocessor());
      break;
    case StageType::MAP_MANAGER:
      stage_ptr.reset(new lidar::MapManager());
      break;
    case StageType::POINT_PILLARS_DETECTION:
      stage_ptr.reset(new lidar::PointPillarsDetection());
      break;
    case StageType::CNN_SEGMENTATION:
      stage_ptr.reset(new lidar::CNNSegmentation());
      break;
    case StageType::NCUT_SEGMENTATION:
      stage_ptr.reset(new lidar::NCutSegmentation());
      break;
    case StageType::MASK_PILLARS_DETECTION:
      stage_ptr.reset(new lidar::MaskPillarsDetection());
      break;
    case StageType::CENTER_POINT_DETECTION:
      stage_ptr.reset(new lidar::CenterPointDetection());
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
    case StageType::CAMERA_DETECTION_POSTPROCESSOR:
      stage_ptr.reset(new camera::CameraDetectionPostprocessor());
      break;
    case StageType::SMOKE_OBSTACLE_DETECTION:
      stage_ptr.reset(new camera::SmokeObstacleDetector());
      break;
    case StageType::CAMERA_DETECTION_PREPROCESSOR:
      stage_ptr.reset(new camera::CameraDetectionPreprocessor());
      break;
    case StageType::OMT_OBSTACLE_TRACKER:
      stage_ptr.reset(new camera::OMTObstacleTracker());
      break;
    case StageType::ALL_LATEST_FUSION:
      stage_ptr.reset(new fusion::AllLatestFusion());
      break;
    case StageType::PROBABILISTIC_FUSION:
      stage_ptr.reset(new fusion::ProbabilisticFusion());
      break;
    case StageType::COLLECT_FUSED_OBJECT:
      stage_ptr.reset(new fusion::CollectFusedObject());
      break;
    case StageType::MULTI_CUE_OBSTACLE_TRANSFORMER:
      stage_ptr.reset(new camera::MultiCueObstacleTransformer());
      break;
    case StageType::LOCATION_REFINER_OBSTACLE_POSTPROCESSOR:
      stage_ptr.reset(new camera::LocationRefinerObstaclePostprocessor());
      break;
    case StageType::BEV_OBSTACLE_DETECTOR:
      stage_ptr.reset(new camera::BEVObstacleDetector());
      break;
    case StageType::OMT_BEV_OBSTACLE_TRACKER:
      stage_ptr.reset(new camera::OMTBEVTracker());
      break;
    case StageType::CADDN_DETECTION:
      stage_ptr.reset(new camera::CaddnObstacleDetector());
      break;
    default:
      return nullptr;
  }

  if (stage_ptr != nullptr) stage_ptr->Init(stage_config_map_[stage_type]);
  return stage_ptr;
}

void Pipeline::Clear() {
  stage_ptrs_.clear();
  stage_config_map_.clear();
}

}  // namespace pipeline
}  // namespace perception
}  // namespace apollo
