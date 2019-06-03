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
#pragma once

#include <string>
#include <vector>

#include "Eigen/StdVector"

#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/fusion_log.h"
#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/fusion/base/sensor_object.h"
#include "modules/perception/fusion/base/track.h"
#include "modules/perception/fusion/common/camera_util.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/probabilities.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/projection_cache.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/track_object_similarity.h"

namespace apollo {
namespace perception {
namespace fusion {

struct TrackObjectDistanceOptions {
  Eigen::Vector3d* ref_point = nullptr;
};  // struct TrackedObjectDistanceOptions

class TrackObjectDistance {
 public:
  TrackObjectDistance() = default;
  ~TrackObjectDistance() = default;
  TrackObjectDistance(const TrackObjectDistance&) = delete;
  TrackObjectDistance operator=(const TrackObjectDistance&) = delete;

  void set_distance_thresh(const float distance_thresh) {
    distance_thresh_ = distance_thresh;
  }
  void ResetProjectionCache(std::string sensor_id, double timestamp) {
    projection_cache_.Reset(sensor_id, timestamp);
  }

  // @brief: compute the distance between input fused track and sensor object
  // @params [in] fused_track: maintained fused track
  // @params [in] sensor_object: sensor observation
  // @params [in] options: options of track object distanace computation
  // @return track object distance
  float Compute(const TrackPtr& fused_track,
                const SensorObjectPtr& sensor_object,
                const TrackObjectDistanceOptions& options);
  // @brief: calculate the similarity between velodyne64 observation and
  // camera observation
  // @return the similarity which belongs to [0, 1]. When velodyne64
  // observation is similar to the camera one, the similarity would
  // close to 1. Otherwise, it would close to 0.
  // @NOTE: original method name is compute_velodyne64_camera_dist_score
  double ComputeLidarCameraSimilarity(const SensorObjectConstPtr& lidar,
                                      const SensorObjectConstPtr& camera,
                                      const bool measurement_is_lidar);
  // @brief: calculate the similarity between radar observation and
  // camera observation
  // @retrun the similarity which belongs to [0, 1]. When radar
  // observation is similar to the camera one, the similarity would
  // close to 1. Otherwise, it would close to 0.
  // @NOTE: original method name is compute_radar_camera_dist_score
  double ComputeRadarCameraSimilarity(const SensorObjectConstPtr& radar,
                                      const SensorObjectConstPtr& camera);

 protected:
  // @brief: compute the distance between lidar observation and
  // lidar observation
  // @return distance of lidar vs. lidar
  float ComputeLidarLidar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos, int range = 3);
  // @brief: compute the distance between lidar observation and
  // radar observation
  // @return distance of lidar vs. radar
  float ComputeLidarRadar(const SensorObjectConstPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos, int range = 3);
  // @brief: compute the distance between radar observation and
  // radar observation
  // @return distance of radar vs. radar
  float ComputeRadarRadar(const SensorObjectPtr& fused_object,
                          const SensorObjectPtr& sensor_object,
                          const Eigen::Vector3d& ref_pos, int range = 3);
  // @brief: compute the distance between lidar observation and
  // camera observation
  // @return distance of lidar vs. camera
  float ComputeLidarCamera(const SensorObjectConstPtr& lidar,
                           const SensorObjectConstPtr& camera,
                           const bool measurement_is_lidar,
                           const bool is_track_id_consistent);
  // @brief: compute the distance between radar observation and
  // camera observation
  // @return distance of radar vs. camera
  float ComputeRadarCamera(const SensorObjectConstPtr& radar,
                           const SensorObjectConstPtr& camera);
  // @brief: compute the distance between camera observation and
  // camera observation
  // @return distance of camera vs. camera
  float ComputeCameraCamera(const SensorObjectPtr& fused_camera,
                            const SensorObjectPtr& sensor_camera);

  // @brief: compute 3d distance between fused object and sensor object
  // @return 3d distance betwene fused object and sensor object
  float ComputePolygonDistance3d(const SensorObjectConstPtr& fused_object,
                                 const SensorObjectPtr& sensor_object,
                                 const Eigen::Vector3d& ref_pos, int range);
  // @brief: compute euclidean distance
  // @return euclidean distance of input pts
  float ComputeEuclideanDistance(const Eigen::Vector3d& des,
                                 const Eigen::Vector3d& src);
  // @brief: compute polygon center
  // @params [out] center: center of input polygon
  // @return true if get center successfully, otherwise return false
  bool ComputePolygonCenter(const base::PolygonDType& polygon,
                            Eigen::Vector3d* center);
  // @brief: compute polygon center
  // @params [out] center: center of input polygon
  // @return true if get center successfully, otherwise return false
  bool ComputePolygonCenter(const base::PolygonDType& polygon,
                            const Eigen::Vector3d& ref_pos, int range,
                            Eigen::Vector3d* center);

 private:
  base::BaseCameraModelPtr QueryCameraModel(const SensorObjectConstPtr& camera);
  bool QueryWorld2CameraPose(const SensorObjectConstPtr& camera,
                             Eigen::Matrix4d* pose);
  bool QueryLidar2WorldPose(const SensorObjectConstPtr& lidar,
                            Eigen::Matrix4d* pose);
  void QueryProjectedVeloCtOnCamera(const SensorObjectConstPtr& velodyne64,
                                    const SensorObjectConstPtr& camera,
                                    const Eigen::Matrix4d& lidar2camera_pose,
                                    Eigen::Vector3d* projected_ct);
  bool QueryPolygonDCenter(const base::ObjectConstPtr& object,
                           const Eigen::Vector3d& ref_pos, const int range,
                           Eigen::Vector3d* polygon_ct);
  void GetModified2DRadarBoxVertices(
      const std::vector<Eigen::Vector3d>& radar_box_vertices,
      const SensorObjectConstPtr& camera,
      const base::BaseCameraModelPtr& camera_intrinsic,
      const Eigen::Matrix4d& world2camera_pose,
      std::vector<Eigen::Vector2d>* radar_box2d_vertices);
  ProjectionCacheObject* BuildProjectionCacheObject(
      const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera,
      const base::BaseCameraModelPtr& camera_model,
      const std::string& measurement_sensor_id, double measurement_timestamp,
      const std::string& projection_sensor_id, double projection_timestamp);
  ProjectionCacheObject* QueryProjectionCacheObject(
      const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera,
      const base::BaseCameraModelPtr& camera_model,
      const bool measurement_is_lidar);
  bool IsTrackIdConsistent(const SensorObjectConstPtr& object1,
                           const SensorObjectConstPtr& object2);
  bool LidarCameraCenterDistanceExceedDynamicThreshold(
      const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera);

  ProjectionCache projection_cache_;
  float distance_thresh_ = 4.0f;
  const float vc_similarity2distance_penalize_thresh_ = 0.07f;
  const float vc_diff2distance_scale_factor_ = 0.8f;
  const float rc_similarity2distance_penalize_thresh_ = 0.1f;
  const float rc_x_similarity_params_2_welsh_loss_scale_ = 0.5f;
  const Eigen::Vector2d rc_min_box_size_ = Eigen::Vector2d(25, 25);

  XSimilarityParams rc_x_similarity_params_ = XSimilarityParams();
  YSimilarityParams rc_y_similarity_params_ = YSimilarityParams();
  HSimilarityParams rc_h_similarity_params_ = HSimilarityParams();
  WSimilarityParams rc_w_similarity_params_ = WSimilarityParams();
  LocSimilarityParams rc_loc_similarity_params_ = LocSimilarityParams();
  XSimilarityParams rc_x_similarity_params_2_ = XSimilarityParams();

 private:
  static double s_lidar2lidar_association_center_dist_threshold_;
  static double s_lidar2radar_association_center_dist_threshold_;
  static double s_radar2radar_association_center_dist_threshold_;
  static size_t s_lidar2camera_projection_downsample_target_pts_num_;
  static size_t s_lidar2camera_projection_vertices_check_pts_num_;
};  // class TrackObjectDistance

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
