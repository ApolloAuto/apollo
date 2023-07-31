/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/tracker/pbf_tracker/pbf_tracker.h"

#include "modules/perception/multi_sensor_fusion/proto/pbf_tracker_config.pb.h"

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/util.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

std::string PbfTracker::s_type_fusion_method_ = "DstTypeFusion";  // NOLINT
std::string PbfTracker::s_existence_fusion_method_ =              // NOLINT
    "DstExistenceFusion";
std::string PbfTracker::s_motion_fusion_method_ =  // NOLINT
    "KalmanMotionFusion";
std::string PbfTracker::s_shape_fusion_method_ = "PbfShapeFusion";  // NOLINT

bool PbfTracker::InitParams(const TrackerInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  PbfTrackerConfig params;
  if (!cyber::common::GetProtoFromFile(config_file, &params)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }

  AINFO << "Load PbfTrackerConfig: " << params.DebugString();

  s_type_fusion_method_ = params.type_fusion_param().name();
  s_motion_fusion_method_ = params.motion_fusion_param().name();
  s_shape_fusion_method_ = params.shape_fusion_param().name();
  s_existence_fusion_method_ = params.existence_fusion_param().name();

  TypeFusionInitOptions type_fusion_init_options;
  auto type_fusion_param = params.type_fusion_param();
  type_fusion_init_options.config_path = type_fusion_param.config_path();
  type_fusion_init_options.config_file = type_fusion_param.config_file();
  DstTypeFusion::Init(type_fusion_init_options);

  ExistenceFusionInitOptions existence_fusion_init_options;
  auto existence_fusion_param = params.existence_fusion_param();
  existence_fusion_init_options.config_path =
      existence_fusion_param.config_path();
  existence_fusion_init_options.config_file =
      existence_fusion_param.config_file();
  DstExistenceFusion::Init(existence_fusion_init_options);

  return true;
}

bool PbfTracker::InitMethods() {
  if (s_type_fusion_method_ == "DstTypeFusion") {
    type_fusion_.reset(new DstTypeFusion(track_));
  } else {
    AERROR << "Unknown type fusion : " << s_type_fusion_method_;
    return false;
  }

  if (s_motion_fusion_method_ == "KalmanMotionFusion") {
    motion_fusion_.reset(new KalmanMotionFusion(track_));
  } else {
    AERROR << "Unknown motion fusion : " << s_motion_fusion_method_;
    return false;
  }

  if (s_existence_fusion_method_ == "DstExistenceFusion") {
    existence_fusion_.reset(new DstExistenceFusion(track_));
  } else {
    AERROR << "Unknown existence fusion : " << s_existence_fusion_method_;
    return false;
  }

  if (s_shape_fusion_method_ == "PbfShapeFusion") {
    shape_fusion_.reset(new PbfShapeFusion(track_));
  } else {
    AERROR << "Unknown shape fusion : " << s_shape_fusion_method_;
    return false;
  }

  return true;
}

bool PbfTracker::Init(TrackPtr track, SensorObjectPtr measurement) {
  track_ = track;
  if (!InitMethods()) {
    return false;
  }
  motion_fusion_->Init();
  return true;
}

void PbfTracker::UpdateWithMeasurement(const TrackerOptions& options,
                                       const SensorObjectPtr measurement,
                                       double target_timestamp) {
  std::string sensor_id = measurement->GetSensorId();
  ADEBUG << "fusion_updating..." << track_->GetTrackId() << " with "
         << sensor_id << "..." << measurement->GetBaseObject()->track_id << "@"
         << FORMAT_TIMESTAMP(measurement->GetTimestamp());
  existence_fusion_->UpdateWithMeasurement(measurement, target_timestamp,
                                           options.match_distance);
  motion_fusion_->UpdateWithMeasurement(measurement, target_timestamp);
  shape_fusion_->UpdateWithMeasurement(measurement, target_timestamp);
  type_fusion_->UpdateWithMeasurement(measurement, target_timestamp);
  track_->UpdateWithSensorObject(measurement);
}

void PbfTracker::UpdateWithoutMeasurement(const TrackerOptions& options,
                                          const std::string& sensor_id,
                                          double measurement_timestamp,
                                          double target_timestamp) {
  existence_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                              target_timestamp,
                                              options.match_distance);
  motion_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                           target_timestamp);
  shape_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                          target_timestamp);
  type_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                         target_timestamp,
                                         options.match_distance);
  track_->UpdateWithoutSensorObject(sensor_id, measurement_timestamp);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
