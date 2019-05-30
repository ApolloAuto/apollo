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
#include "modules/perception/fusion/lib/data_fusion/existance_fusion/dst_existance_fusion/dst_existance_fusion.h"

#include <limits>

#include "boost/format.hpp"

#include "cyber/common/file.h"
#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/common/camera_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/proto/dst_existance_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

const char *DstExistanceFusion::name_ = "DstExistanceFusion";
const char *DstExistanceFusion::toic_name_ = "DstToicFusion";
ExistanceDstMaps DstExistanceFusion::existance_dst_maps_;
ToicDstMaps DstExistanceFusion::toic_dst_maps_;
DstExistanceFusionOptions DstExistanceFusion::options_;

DstExistanceFusion::DstExistanceFusion(TrackPtr track)
    : BaseExistanceFusion(track),
      fused_toic_(toic_name_),
      fused_existance_(name_) {}

bool DstExistanceFusion::Init() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("DstExistanceFusion", &options)) {
    AERROR << "GetFusionInitOptions failed ";
    return false;
  }

  std::string woork_root_config = GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  DstExistanceFusionConfig params;

  if (!cyber::common::GetProtoFromFile(config, &params)) {
    AERROR << "Read config failed: " << config;
    return false;
  }
  for (auto valid_dist : params.camera_valid_dist()) {
    std::string camera_id = valid_dist.camera_name();
    options_.camera_max_valid_dist_[camera_id] = valid_dist.valid_dist();
    AINFO << "dst existence fusion params: " << camera_id
          << " max valid dist: " << options_.camera_max_valid_dist_[camera_id];
  }

  options_.track_object_max_match_distance_ =
      params.track_object_max_match_distance();
  AINFO << "dst existence fusion params: "
        << " track_object_max_match_distance: "
        << options_.track_object_max_match_distance_;

  DstManager::Instance()->AddApp(name_, existance_dst_maps_.fod_subsets_,
                                 existance_dst_maps_.subset_names_);

  DstManager::Instance()->AddApp(toic_name_, toic_dst_maps_.fod_subsets_,
                                 toic_dst_maps_.subset_names_);

  return DstManager::Instance()->IsAppAdded(name_) &&
         DstManager::Instance()->IsAppAdded(toic_name_);
}

void DstExistanceFusion::UpdateWithMeasurement(
    const SensorObjectPtr measurement, double target_timestamp,
    double match_dist) {
  std::string sensor_id = measurement->GetSensorId();
  double timestamp = measurement->GetTimestamp();
  double max_match_distance = options_.track_object_max_match_distance_;
  double association_prob = 1 - match_dist / max_match_distance;
  if (IsCamera(measurement)) {
    association_prob = 1.0;
  }
  base::ObjectConstPtr obj = measurement->GetBaseObject();
  double exist_factor = GetExistReliability(measurement);
  double decay = ComputeDistDecay(obj, sensor_id, timestamp);
  if (IsRadar(measurement)) {
    decay = ComputeFeatureInfluence(measurement);
  }
  double obj_exist_prob = exist_factor * decay;
  Dst existance_evidence(fused_existance_.Name());
  existance_evidence.SetBba(
      {{ExistanceDstMaps::EXIST, obj_exist_prob},
       {ExistanceDstMaps::EXISTUNKOWN, 1 - obj_exist_prob}});
  // TODO(all) hard code for fused exist bba
  const double exist_fused_w = 1.0;
  ADEBUG << " before update exist prob: " << GetExistanceProbability();
  fused_existance_ =
      fused_existance_ + existance_evidence * exist_fused_w * association_prob;
  ADEBUG << " update with, EXIST prob: " << GetExistanceProbability()
         << " obj_id " << measurement->GetBaseObject()->track_id
         << " association_prob: " << association_prob << " decay: " << decay
         << " exist_prob: " << obj_exist_prob
         << " sensor_id: " << measurement->GetSensorId()
         << " track_id: " << track_ref_->GetTrackId();
  if (IsCamera(measurement)) {
    UpdateToicWithCameraMeasurement(measurement, match_dist);
  }
  UpdateExistanceState();
}

void DstExistanceFusion::UpdateWithoutMeasurement(const std::string &sensor_id,
                                                  double measurement_timestamp,
                                                  double target_timestamp,
                                                  double min_match_dist) {
  SensorObjectConstPtr camera_object = nullptr;
  if (common::SensorManager::Instance()->IsCamera(sensor_id)) {
    camera_object = track_ref_->GetSensorObject(sensor_id);
    UpdateToicWithoutCameraMeasurement(sensor_id, measurement_timestamp,
                                       min_match_dist);
  }
  SensorObjectConstPtr lidar_object = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr camera_object_latest =
      track_ref_->GetLatestCameraObject();
  SensorObjectConstPtr radar_object = track_ref_->GetLatestRadarObject();
  if ((lidar_object != nullptr && lidar_object->GetSensorId() == sensor_id) ||
      (camera_object_latest != nullptr &&
       camera_object_latest->GetSensorId() == sensor_id) ||
      (radar_object != nullptr && radar_object->GetSensorId() == sensor_id &&
       lidar_object == nullptr && camera_object_latest == nullptr)) {
    Dst existance_evidence(fused_existance_.Name());
    double unexist_factor = GetUnexistReliability(sensor_id);
    base::ObjectConstPtr obj = track_ref_->GetFusedObject()->GetBaseObject();
    double dist_decay = ComputeDistDecay(obj, sensor_id, measurement_timestamp);
    double obj_unexist_prob = unexist_factor * dist_decay;
    existance_evidence.SetBba(
        {{ExistanceDstMaps::NEXIST, obj_unexist_prob},
         {ExistanceDstMaps::EXISTUNKOWN, 1 - obj_unexist_prob}});
    // TODO(all) hard code for fused exist bba
    const double unexist_fused_w = 1.0;
    double min_match_dist_score = min_match_dist;
    // if (!sensor_manager->IsCamera(sensor_id)) {
    //   min_match_dist_score = std::max(1 - min_match_dist /
    //   options_.track_object_max_match_distance_, 0.0);
    // }
    ADEBUG << " before update exist prob: " << GetExistanceProbability()
           << " min_match_dist: " << min_match_dist
           << " min_match_dist_score: " << min_match_dist_score;
    fused_existance_ = fused_existance_ + existance_evidence * unexist_fused_w *
                                              (1 - min_match_dist_score);
    ADEBUG << " update without, EXIST prob: " << GetExistanceProbability()
           << " 1 - match_dist_score: " << 1 - min_match_dist_score
           << " sensor_id: " << sensor_id << " dist_decay: " << dist_decay
           << " track_id: " << track_ref_->GetTrackId();
  }
  UpdateExistanceState();
}

double DstExistanceFusion::ComputeDistDecay(base::ObjectConstPtr obj,
                                            const std::string &sensor_id,
                                            double timestamp) {
  double distance = (std::numeric_limits<float>::max)();
  double dist_decay = 1.0;
  Eigen::Affine3d sensor2world_pose;
  bool status = SensorDataManager::Instance()->GetPose(sensor_id, timestamp,
                                                       &sensor2world_pose);
  if (!status) {
    AERROR << "Failed to get pose";
    return dist_decay;
  }
  Eigen::Matrix4d world2sensor_pose = sensor2world_pose.matrix().inverse();
  if (!world2sensor_pose.allFinite()) {
    AERROR << boost::format(
                  "The obtained camera pose is invalid. sensor_id : %s"
                  " timestamp %16.6f") %
                  sensor_id % timestamp;
    return dist_decay;
  }
  Eigen::Vector3d obj_ct = obj->center;
  Eigen::Vector3d obj_ct_local =
      (world2sensor_pose * obj_ct.homogeneous()).head(3);
  distance = std::sqrt(obj_ct_local.cwiseProduct(obj_ct_local).sum());
  if (distance > 60) {
    dist_decay = 0.8;
  }
  return dist_decay;
}

double DstExistanceFusion::ComputeFeatureInfluence(
    const SensorObjectPtr measurement) {
  double velocity = measurement->GetBaseObject()->velocity.norm();
  auto sigmoid_fun = [](double velocity) {
    return 1.0 / (1.0 + exp(-velocity));
  };
  double velocity_fact = sigmoid_fun(velocity);
  velocity_fact = velocity > 4.0 ? velocity_fact : 0.0;
  double confidence = measurement->GetBaseObject()->confidence;
  ADEBUG << " sensor_id: " << measurement->GetSensorId()
         << " velocity: " << velocity << " velocity_fact: " << velocity_fact
         << " confidence: " << confidence;
  return velocity_fact * confidence;
}

double DstExistanceFusion::GetExistReliability(
    const SensorObjectPtr measurement) {
  bool unknown =
      (measurement->GetBaseObject()->type == base::ObjectType::UNKNOWN ||
       measurement->GetBaseObject()->type == base::ObjectType::UNKNOWN_MOVABLE);
  double unknown_ratio = unknown ? 0.6 : 1.0;
  common::SensorManager *sensor_manager = common::SensorManager::Instance();
  CHECK_NOTNULL(sensor_manager);
  if (sensor_manager->IsCamera(measurement->GetSensorId())) {
    return 0.8 * unknown_ratio;
  }
  if (sensor_manager->IsLidar(measurement->GetSensorId())) {
    return 0.9 * unknown_ratio;
  }
  return 0.6;
}

double DstExistanceFusion::GetUnexistReliability(const std::string &sensor_id) {
  common::SensorManager *sensor_manager = common::SensorManager::Instance();
  CHECK_NOTNULL(sensor_manager);
  if (sensor_manager->IsCamera(sensor_id)) {
    return 0.8;
  }
  if (sensor_manager->IsLidar(sensor_id)) {
    return 0.9;
  }
  return 0.6;
}

void DstExistanceFusion::UpdateToicWithoutCameraMeasurement(
    const std::string &sensor_id, double measurement_timestamp,
    double min_match_dist) {
  double dist_score = min_match_dist;
  double in_view_ratio = 0.0;
  // 1.get camera intrinsic and pose
  SensorDataManager *sensor_manager = SensorDataManager::Instance();
  CHECK(sensor_manager != nullptr) << "Failed to get sensor manager";

  base::BaseCameraModelPtr camera_model =
      sensor_manager->GetCameraIntrinsic(sensor_id);
  CHECK(camera_model != nullptr)
      << "Failed to get camera intrinsic for " << sensor_id;

  Eigen::Affine3d sensor2world_pose;
  bool status = sensor_manager->GetPose(sensor_id, measurement_timestamp,
                                        &sensor2world_pose);
  auto max_dist_it = options_.camera_max_valid_dist_.find(sensor_id);
  if (max_dist_it == options_.camera_max_valid_dist_.end()) {
    AWARN << boost::format(
                 "There is no pre-defined max valid camera"
                 " dist for sensor type: %s") %
                 sensor_id;
  }
  if (status && max_dist_it != options_.camera_max_valid_dist_.end()) {
    SensorObjectConstPtr lidar_object = track_ref_->GetLatestLidarObject();
    SensorObjectConstPtr radar_object = track_ref_->GetLatestRadarObject();
    double camera_max_dist = max_dist_it->second;
    if (lidar_object != nullptr) {
      in_view_ratio = ObjectInCameraView(
          lidar_object, camera_model, sensor2world_pose, measurement_timestamp,
          camera_max_dist, true, false);
    } else if (radar_object != nullptr) {
      in_view_ratio = ObjectInCameraView(
          radar_object, camera_model, sensor2world_pose, measurement_timestamp,
          camera_max_dist, false, false);
    }
  }

  Dst toic_evidence(fused_toic_.Name());
  toic_evidence.SetBba({{ToicDstMaps::NTOIC, 1 - dist_score},
                        {ToicDstMaps::TOICUNKOWN, dist_score}});
  // TODO(yuantingrong): hard code for fused toic bba
  const double toic_fused_w = 1.0;
  fused_toic_ = fused_toic_ + toic_evidence * in_view_ratio * toic_fused_w;
}

void DstExistanceFusion::UpdateToicWithCameraMeasurement(
    const SensorObjectPtr &camera_obj, double match_dist) {
  std::string sensor_id = camera_obj->GetSensorId();
  double timestamp = camera_obj->GetTimestamp();
  double in_view_ratio = 0.0;
  // 1.get camera intrinsic and pose
  SensorDataManager *sensor_manager = SensorDataManager::Instance();

  base::BaseCameraModelPtr camera_model =
      sensor_manager->GetCameraIntrinsic(sensor_id);
  CHECK(camera_model != nullptr)
      << "Failed to get camera intrinsic for " << sensor_id;

  Eigen::Affine3d sensor2world_pose;
  bool status =
      sensor_manager->GetPose(sensor_id, timestamp, &sensor2world_pose);
  auto max_dist_it = options_.camera_max_valid_dist_.find(sensor_id);
  if (max_dist_it == options_.camera_max_valid_dist_.end()) {
    AWARN << boost::format(
                 "There is no pre-defined max valid camera"
                 " dist for sensor type: %s") %
                 sensor_id;
  } else {
    ADEBUG << "camera dist: " << sensor_id << " " << max_dist_it->second;
  }
  if (status && max_dist_it != options_.camera_max_valid_dist_.end()) {
    SensorObjectConstPtr lidar_object = track_ref_->GetLatestLidarObject();
    SensorObjectConstPtr radar_object = track_ref_->GetLatestRadarObject();
    double camera_max_dist = max_dist_it->second;
    if (lidar_object != nullptr) {
      in_view_ratio =
          ObjectInCameraView(lidar_object, camera_model, sensor2world_pose,
                             timestamp, camera_max_dist, true, false);
    } else if (radar_object != nullptr) {
      in_view_ratio =
          ObjectInCameraView(radar_object, camera_model, sensor2world_pose,
                             timestamp, camera_max_dist, false, false);
    }
  }

  double max_match_distance = options_.track_object_max_match_distance_;
  double association_prob = 1 - match_dist / max_match_distance;

  Dst toic_evidence(fused_toic_.Name());
  toic_evidence.SetBba({{ToicDstMaps::TOIC, association_prob},
                        {ToicDstMaps::TOICUNKOWN, 1 - association_prob}});
  // TODO(yuantingrong): hard code for fused toic bba
  const double toic_fused_w = 0.7;
  fused_toic_ = fused_toic_ + toic_evidence * toic_fused_w * in_view_ratio;
}

std::string DstExistanceFusion::Name() const { return name_; }

double DstExistanceFusion::GetExistanceProbability() const {
  size_t toic_ind = DstManager::Instance()->FodSubsetToInd(
      fused_existance_.Name(), ExistanceDstMaps::EXIST);
  fused_existance_.ComputeProbability();
  const std::vector<double> &existance_probs_vec =
      fused_existance_.GetProbabilityVec();
  return existance_probs_vec[toic_ind];
}

double DstExistanceFusion::GetToicProbability() const {
  size_t toic_ind = DstManager::Instance()->FodSubsetToInd(fused_toic_.Name(),
                                                           ToicDstMaps::TOIC);
  fused_toic_.ComputeProbability();
  const std::vector<double> &toic_probs_vec = fused_toic_.GetProbabilityVec();
  return toic_probs_vec[toic_ind];
}

void DstExistanceFusion::UpdateExistanceState() {
  double toic_p = GetToicProbability();
  track_ref_->SetToicProb(toic_p);
  double existance_p = GetExistanceProbability();
  track_ref_->SetExistanceProb(existance_p);
  // note, here max_p > 0.5, min_p < 0.5
  auto scale_probability = [](double p, double max_p, double min_p) {
    if (p > 0.5) {
      p = 0.5 + (p - 0.5) * (max_p - 0.5) / 0.5;
    } else {
      p = 0.5 - (0.5 - p) * (0.5 - min_p) / 0.5;
    }
    return p;
  };
  // TODO(yuantingrong): hard code
  const double max_p = 0.8;
  const double min_p = 0.2;
  double toic_score = scale_probability(toic_p, max_p, min_p);
  // when this fused object have lidar object, just return 1.0
  // which means wen do not want introducing historical information
  // to affect the association, but when this fused object have just
  // radar object, we want using the historical information to filter
  // large amount of false positive
  if (!(track_ref_->GetLidarObjects()).empty()) {
    toic_score_ = 0.5;
  } else if (!track_ref_->GetRadarObjects().empty()) {
    toic_score_ = toic_score;
  } else if (!track_ref_->GetCameraObjects().empty()) {
    toic_score_ = 0.5;
  } else {
    AERROR << boost::format("this fused object: %d has no sensor objects") %
                  track_ref_->GetTrackId();
    toic_score_ = 0.5;
  }
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
