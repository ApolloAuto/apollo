/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar4d_detection/lib/preprocessor/radar_preprocessor.h"

#include "modules/perception/radar4d_detection/proto/preprocessor_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace radar4d {

const float RadarPreprocessor::kPointInfThreshold = 1e3;
int RadarPreprocessor::current_idx_ = 0;
std::unordered_map<int, int> RadarPreprocessor::local2global_;

bool RadarPreprocessor::Init(const PreprocessorInitOptions& options) {
  // load config file
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  PreprocessorConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  rcs_offset_ = config.rcs_offset();
  return true;
}

bool RadarPreprocessor::Preprocess(
    const std::shared_ptr<apollo::drivers::OculiiPointCloud const>& message,
    const PreprocessorOptions& options,
    RadarFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  if (frame->cloud == nullptr) {
    frame->cloud = base::RadarPointFCloudPool::Instance().Get();
  }
  if (frame->world_cloud == nullptr) {
    frame->world_cloud = base::RadarPointDCloudPool::Instance().Get();
  }

  frame->cloud->set_timestamp(message->measurement_time());
  if (message->point_size() > 0) {
    frame->cloud->reserve(message->point_size());
    base::RadarPointF point;
    for (int i = 0; i < message->point_size(); ++i) {
      // original point cloud from driver message
      const apollo::drivers::OculiiPointXYZIV& pt = message->point(i);
      const apollo::drivers::OculiiRawPointcloud& raw_pt =
        message->raw_pointclouds(i);
      // filter abnormal points
      if (filter_naninf_points_) {
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
          continue;
        }
        if (fabs(pt.x()) > kPointInfThreshold ||
            fabs(pt.y()) > kPointInfThreshold ||
            fabs(pt.z()) > kPointInfThreshold) {
          continue;
        }
      }
      // filter points higher than the threshold
      if (filter_high_z_points_ && pt.z() > z_threshold_) {
        continue;
      }
      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();
      // rcs offset is set for different radars used for training and testing
      point.rcs = static_cast<float>(pt.intensity() + rcs_offset_);
      // doppler velocity is returned by radar
      point.velocity = static_cast<float>(raw_pt.doppler());
      // compensated velocity of each point needs to be calculated
      point.comp_vel = CalCompensatedVelocity(point, options);
      frame->cloud->push_back(point);
    }
    // transform the cloud point from local to world
    TransformCloud(frame->cloud, frame->radar2world_pose, frame->world_cloud);
  }
  return true;
}

bool RadarPreprocessor::TransformCloud(
    const base::RadarPointFCloudPtr& local_cloud, const Eigen::Affine3d& pose,
    base::RadarPointDCloudPtr world_cloud) const {
  if (local_cloud == nullptr) {
    return false;
  }
  world_cloud->clear();
  world_cloud->reserve(local_cloud->size());
  for (size_t i = 0; i < local_cloud->size(); ++i) {
    auto& pt = local_cloud->at(i);
    Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
    trans_point = pose * trans_point;
    base::RadarPointD world_point;
    world_point.x = trans_point(0);
    world_point.y = trans_point(1);
    world_point.z = trans_point(2);
    world_point.rcs = pt.rcs;
    world_point.velocity = pt.velocity;
    world_point.comp_vel = pt.comp_vel;
    world_cloud->push_back(world_point);
  }
  return true;
}

float RadarPreprocessor::CalCompensatedVelocity(
  const base::RadarPointF& point,
  const PreprocessorOptions& options) {
  // transfer the ego speed from world coor to radar coor
  const Eigen::Matrix4d& radar2world = *(options.radar2world_pose);
  Eigen::Matrix3d radar2world_rotate = radar2world.block<3, 3>(0, 0);
  Eigen::Vector3d radar_speed =
      static_cast<Eigen::Matrix<double, 3, 1, 0, 3, 1>>(
      radar2world_rotate.inverse() *
      options.car_linear_speed.cast<double>());

  // calculate the direction of point
  Eigen::Vector3f local_loc(point.x, point.y, point.z);
  float azimuth = std::atan2(local_loc[1], local_loc[0]);
  float xy_temp = std::sqrt(local_loc[0] * local_loc[0]
                          + local_loc[1] * local_loc[1]);
  float elevation = std::atan2(local_loc[2], xy_temp);

  // calculate the compensated velocity
  // = doppler velocity + projection of ego velocity
  float compensated_v = point.velocity +
                        radar_speed[0] * std::cos(azimuth) +
                        radar_speed[1] * std::sin(azimuth) +
                        radar_speed[2] * std::sin(elevation);
  return compensated_v;
}

PERCEPTION_REGISTER_PREPROCESSOR(RadarPreprocessor);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
