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
#include "modules/perception/obstacle/onboard/obstacle_perception.h"

#include <string>

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.h"
#include "modules/perception/obstacle/radar/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/radar/modest/modest_radar_detector.h"

DEFINE_string(obstacle_show_type, "fused",
              "show obstacle from lidar/radar/fused");

namespace apollo {
namespace perception {

ObstacleShowType GetObstacleShowType(const std::string& show_type_string) {
  if (show_type_string == "lidar") {
    return SHOW_LIDAR;
  } else if (show_type_string == "radar") {
    return SHOW_RADAR;
  } else if (show_type_string == "fused") {
    return SHOW_FUSED;
  } else {
    return MAX_SHOW_TYPE;
  }
}

ObstaclePerception::ObstaclePerception() : lidar_pose_inited_(false) {}

ObstaclePerception::~ObstaclePerception() {}

bool ObstaclePerception::Init() {
  RegistAllAlgorithm();
  /// initialize lidar detector
  lidar_perception_.reset(new LidarProcess);
  if (lidar_perception_ == nullptr) {
    AERROR << "Failed to get LidarProcess instance";
    return false;
  }
  if (!lidar_perception_->Init()) {
    AERROR << "Failed to initialize lidar perception";
    return false;
  }
  /// initialize radar detector
  radar_detector_.reset(BaseRadarDetectorRegisterer::GetInstanceByName(
      FLAGS_onboard_radar_detector));
  if (radar_detector_ == nullptr) {
    AERROR << "Failed to get RadarDetector plugin "
           << FLAGS_onboard_radar_detector;
    return false;
  }
  if (!radar_detector_->Init()) {
    AERROR << "Failed to initialize RadarDetector : "
           << FLAGS_onboard_radar_detector;
  }
  /// initialize fusion
  fusion_.reset(BaseFusionRegisterer::GetInstanceByName(FLAGS_onboard_fusion));
  if (fusion_ == nullptr) {
    AERROR << "Failed to get fusion instance: " << FLAGS_onboard_fusion;
    return false;
  }
  if (!fusion_->Init()) {
    AERROR << "Failed to init fusion:" << FLAGS_onboard_fusion;
    return false;
  }
  /// initialize visualizer
  if (FLAGS_enable_visualization) {
    frame_visualizer_.reset(new OpenglVisualizer());
    if (frame_visualizer_ == nullptr) {
      AERROR << "Failed to get OpenglVisualizer instance";
      return false;
    }
    if (!frame_visualizer_->Init()) {
      AERROR << "Failed to init visualizer.";
      return false;
    }
    obstacle_show_type_ = GetObstacleShowType(FLAGS_obstacle_show_type);
    if (obstacle_show_type_ == MAX_SHOW_TYPE) {
      AERROR << "Failed to get ObstacleShowType";
      return false;
    }
  }

  return true;
}

void ObstaclePerception::RegistAllAlgorithm() {
  RegisterFactoryDummyRadarDetector();

  RegisterFactoryModestRadarDetector();
  RegisterFactoryProbabilisticFusion();
}

bool ObstaclePerception::Process(
    SensorRawFrame* frame, std::vector<std::shared_ptr<Object>>* out_objects) {
  if (frame == nullptr || out_objects == nullptr) {
    return false;
  }
  PERF_BLOCK_START();

  std::shared_ptr<SensorObjects> sensor_objects(new SensorObjects());
  if (frame->sensor_type_ == SensorType::VELODYNE_64) {
    /// lidar obstacle detection
    VelodyneRawFrame* velodyne_frame = dynamic_cast<VelodyneRawFrame*>(frame);
    std::shared_ptr<Eigen::Matrix4d> velodyne_pose(new Eigen::Matrix4d);
    *(velodyne_pose.get()) = velodyne_frame->pose_;
    if (!lidar_perception_->Process(velodyne_frame->timestamp_,
                                    velodyne_frame->cloud_, velodyne_pose)) {
      AERROR << "Lidar perception error!, " << std::fixed
             << std::setprecision(12) << velodyne_frame->timestamp_;
      return false;
    }
    sensor_objects->objects = lidar_perception_->GetObjects();
    AINFO << "lidar objects size: " << sensor_objects->objects.size();
    PERF_BLOCK_END("lidar_perception");
    /// set frame content
    if (FLAGS_enable_visualization) {
      frame_content_.SetLidarPose(velodyne_frame->pose_);
      frame_content_.SetLidarCloud(velodyne_frame->cloud_);
      if (obstacle_show_type_ == SHOW_LIDAR) {
        frame_content_.SetTrackedObjects(sensor_objects->objects);
      }
      lidar_pose_inited_ = true;
    }
  } else if (frame->sensor_type_ == SensorType::RADAR) {
    /// radar obstacle detection
    RadarRawFrame* radar_frame = dynamic_cast<RadarRawFrame*>(frame);
    RadarDetectorOptions options;
    options.radar2world_pose = &(radar_frame->pose_);
    options.car_linear_speed = radar_frame->car_linear_speed_;
    std::vector<std::shared_ptr<Object>> objects;
    std::vector<PolygonDType> map_polygons;
    if (!radar_detector_->Detect(radar_frame->raw_obstacles_, map_polygons,
                                 options, &objects)) {
      AERROR << "Radar perception error!, " << std::fixed
             << std::setprecision(12) << radar_frame->timestamp_;
      return false;
    }
    sensor_objects->objects = objects;
    AINFO << "radar objects size: " << objects.size();
    PERF_BLOCK_END("radar_detection");
    /// set frame content
    if (FLAGS_enable_visualization && obstacle_show_type_ == SHOW_RADAR) {
      frame_content_.SetTrackedObjects(sensor_objects->objects);
    }
  } else {
    AERROR << "Unknown sensor type : " << static_cast<int>(frame->sensor_type_);
    return false;
  }
  sensor_objects->sensor_type = frame->sensor_type_;
  sensor_objects->sensor_id = GetSensorType(frame->sensor_type_);
  sensor_objects->timestamp = frame->timestamp_;
  sensor_objects->sensor2world_pose = frame->pose_;

  /// fusion
  std::vector<SensorObjects> multi_sensor_objs;
  multi_sensor_objs.push_back(*sensor_objects);
  std::vector<std::shared_ptr<Object>> fused_objects;
  if (!fusion_->Fuse(multi_sensor_objs, &fused_objects)) {
    AERROR << "Failed to fusion";
    return false;
  }
  *out_objects = fused_objects;
  AINFO << "fused objects size: " << fused_objects.size();
  PERF_BLOCK_END("sensor_fusion");
  /// set frame content
  if (FLAGS_enable_visualization) {
    if (obstacle_show_type_ == SHOW_FUSED) {
      frame_content_.SetTrackedObjects(fused_objects);
      if (frame->sensor_type_ != SensorType::VELODYNE_64) {
        return true;
      }
    }
    if (lidar_pose_inited_) {
      frame_visualizer_->UpdateCameraSystem(&frame_content_);
      frame_visualizer_->Render(frame_content_);
    }
  }

  return true;
}

}  // namespace perception
}  // namespace apollo
