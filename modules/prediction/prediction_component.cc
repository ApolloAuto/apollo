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

#include "modules/prediction/prediction_component.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "boost/filesystem.hpp"
#include "boost/range/iterator_range.hpp"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/prediction/util/data_extraction.h"

namespace apollo {
namespace prediction {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterConfig;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::common::util::DirectoryExists;
using apollo::common::util::Glob;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;

PredictionComponent::~PredictionComponent() { Stop(); }

std::string PredictionComponent::Name() const {
  return FLAGS_prediction_module_name;
}

void PredictionComponent::ProcessOfflineData(const std::string& filename) {
  // TODO(all) implement

  /**
   const std::vector<std::string> topics{FLAGS_perception_obstacle_topic,
                                         FLAGS_localization_topic};
   rosbag::Bag bag;
   try {
     bag.open(filename, rosbag::bagmode::Read);
   } catch (const rosbag::BagIOException& e) {
     AERROR << "BagIOException when open bag: " << filename
            << " Exception: " << e.what();
     bag.close();
     return;
   } catch (...) {
     AERROR << "Failed to open bag: " << filename;
     bag.close();
     return;
   }
   rosbag::View view(bag, rosbag::TopicQuery(topics));
   for (auto it = view.begin(); it != view.end(); ++it) {
     if (it->getTopic() == FLAGS_localization_topic) {
       OnLocalization(*(it->instantiate<LocalizationEstimate>()));
     } else if (it->getTopic() == FLAGS_perception_obstacle_topic) {
       RunOnce(*(it->instantiate<PerceptionObstacles>()));
     }
   }
   bag.close();
   **/
}

bool PredictionComponent::Init() {
  AINFO << "Loading gflag from file: " << ConfigFilePath();
  google::SetCommandLineOption("flagfile", ConfigFilePath().c_str());

  component_start_time_ = Clock::NowInSeconds();

  // Load prediction conf
  prediction_conf_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_prediction_conf_file,
                                      &prediction_conf_)) {
    AERROR << "Unable to load prediction conf file: "
           << FLAGS_prediction_conf_file;
    return false;
  } else {
    ADEBUG << "Prediction config file is loaded into: "
           << prediction_conf_.ShortDebugString();
  }

  adapter_conf_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_prediction_adapter_config_filename,
                                      &adapter_conf_)) {
    AERROR << "Unable to load adapter conf file: "
           << FLAGS_prediction_adapter_config_filename;
    return false;
  } else {
    ADEBUG << "Adapter config file is loaded into: "
           << adapter_conf_.ShortDebugString();
  }

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<ADCTrajectory>& adc_trajectory) {
        ADEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        OnPlanning(*adc_trajectory);
      });

  // Initialization of all managers
  ContainerManager::Instance()->Init(adapter_conf_);
  EvaluatorManager::Instance()->Init(prediction_conf_);
  PredictorManager::Instance()->Init(prediction_conf_);

  if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Map cannot be loaded.";
    return false;
  }

  prediction_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);

  if (FLAGS_prediction_offline_mode) {
    if (!FeatureOutput::Ready()) {
      AERROR << "Feature output is not ready.";
      return false;
    }
    if (FLAGS_prediction_offline_bags.empty()) {
      return true;  // use listen to ROS topic mode
    }
    std::vector<std::string> inputs;
    apollo::common::util::split(FLAGS_prediction_offline_bags, ':', &inputs);
    for (const auto& input : inputs) {
      std::vector<std::string> offline_bags;
      GetDataFileNames(boost::filesystem::path(input), &offline_bags);
      std::sort(offline_bags.begin(), offline_bags.end());
      AINFO << "For input " << input << ", found " << offline_bags.size()
            << "  rosbags to process";
      for (std::size_t i = 0; i < offline_bags.size(); ++i) {
        AINFO << "\tProcessing: [ " << i << " / " << offline_bags.size()
              << " ]: " << offline_bags[i];
        ProcessOfflineData(offline_bags[i]);
      }
    }
    Stop();
    // TODO(kechxu) accord to cybertron
  }
  return true;
}

void PredictionComponent::Stop() {
  if (FLAGS_prediction_offline_mode) {
    FeatureOutput::Close();
  }
}

void PredictionComponent::OnLocalization(
    const LocalizationEstimate& localization) {
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::Instance()->GetContainer(AdapterConfig::LOCALIZATION));
  CHECK_NOTNULL(pose_container);
  pose_container->Insert(localization);

  ADEBUG << "Received a localization message ["
         << localization.ShortDebugString() << "].";
}

void PredictionComponent::OnPlanning(
    const planning::ADCTrajectory& adc_trajectory) {
  ADCTrajectoryContainer* adc_trajectory_container =
      dynamic_cast<ADCTrajectoryContainer*>(
          ContainerManager::Instance()->GetContainer(
              AdapterConfig::PLANNING_TRAJECTORY));
  CHECK_NOTNULL(adc_trajectory_container);
  adc_trajectory_container->Insert(adc_trajectory);

  ADEBUG << "Received a planning message [" << adc_trajectory.ShortDebugString()
         << "].";
}

bool PredictionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles,
    const std::shared_ptr<LocalizationEstimate>& localization) {
  if (FLAGS_prediction_test_mode &&
      (Clock::NowInSeconds() - component_start_time_ >
       FLAGS_prediction_test_duration)) {
    ADEBUG << "Prediction finished running in test mode";
    // TODO(kechxu) accord to cybertron
    // ros::shutdown();
  }

  // Update relative map if needed
  // AdapterManager::Observe();
  if (FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Relative map is empty.";
    return false;
  }

  double start_timestamp = Clock::NowInSeconds();

  OnLocalization(*localization);

  // Insert obstacle
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::Instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(obstacles_container);
  obstacles_container->Insert(*perception_obstacles);

  // Scenario analysis
  ScenarioManager::Instance()->Run();

  // Prioritize obstacles
  obstacles_container->PrioritizeObstacles();

  obstacles_container->BuildLaneGraph();

  ADEBUG << "Received a perception message ["
         << perception_obstacles->ShortDebugString() << "].";

  // Update ADC status
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::Instance()->GetContainer(AdapterConfig::LOCALIZATION));
  ADCTrajectoryContainer* adc_container = dynamic_cast<ADCTrajectoryContainer*>(
      ContainerManager::Instance()->GetContainer(
          AdapterConfig::PLANNING_TRAJECTORY));
  CHECK_NOTNULL(pose_container);
  CHECK_NOTNULL(adc_container);

  PerceptionObstacle* adc = pose_container->ToPerceptionObstacle();
  if (adc != nullptr) {
    obstacles_container->InsertPerceptionObstacle(*adc, adc->timestamp());
    double x = adc->position().x();
    double y = adc->position().y();
    ADEBUG << "Get ADC position [" << std::fixed << std::setprecision(6) << x
           << ", " << std::fixed << std::setprecision(6) << y << "].";
    adc_container->SetPosition({x, y});
  }

  // Make evaluations
  EvaluatorManager::Instance()->Run(*perception_obstacles);

  // No prediction trajectories for offline mode
  if (FLAGS_prediction_offline_mode) {
    return true;
  }

  // Make predictions
  PredictorManager::Instance()->Run(*perception_obstacles);

  auto prediction_obstacles =
      PredictorManager::Instance()->prediction_obstacles();
  prediction_obstacles.set_start_timestamp(start_timestamp);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  prediction_obstacles.mutable_header()->set_lidar_timestamp(
      perception_obstacles->header().lidar_timestamp());
  prediction_obstacles.mutable_header()->set_camera_timestamp(
      perception_obstacles->header().camera_timestamp());
  prediction_obstacles.mutable_header()->set_radar_timestamp(
      perception_obstacles->header().radar_timestamp());

  if (FLAGS_prediction_test_mode) {
    for (auto const& prediction_obstacle :
         prediction_obstacles.prediction_obstacle()) {
      for (auto const& trajectory : prediction_obstacle.trajectory()) {
        for (auto const& trajectory_point : trajectory.trajectory_point()) {
          if (!ValidationChecker::ValidTrajectoryPoint(trajectory_point)) {
            AERROR << "Invalid trajectory point ["
                   << trajectory_point.ShortDebugString() << "]";
            return false;
          }
        }
      }
    }
  }
  prediction_writer_->Write(
      std::make_shared<PredictionObstacles>(prediction_obstacles));
  return true;
}

}  // namespace prediction
}  // namespace apollo
