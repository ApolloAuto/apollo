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
#include "modules/perception/fusion/lib/data_association/hm_data_association/track_object_distance.h"

#include <boost/format.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <utility>

#include "modules/perception/base/camera.h"
#include "modules/perception/base/point.h"
#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/common/geometry/camera_homography.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/chi_squared_cdf_1_0.0500_0.999900.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/chi_squared_cdf_2_0.0500_0.999900.h"

namespace apollo {
namespace perception {
namespace fusion {

double TrackObjectDistance::s_lidar2lidar_association_center_dist_threshold_ =
    10.0;
double TrackObjectDistance::s_lidar2radar_association_center_dist_threshold_ =
    10.0;
double TrackObjectDistance::s_radar2radar_association_center_dist_threshold_ =
    10.0;
size_t
    TrackObjectDistance::s_lidar2camera_projection_downsample_target_pts_num_ =
        100;
size_t TrackObjectDistance::s_lidar2camera_projection_vertices_check_pts_num_ =
    20;

void TrackObjectDistance::GetModified2DRadarBoxVertices(
    const std::vector<Eigen::Vector3d>& radar_box_vertices,
    const SensorObjectConstPtr& camera,
    const base::BaseCameraModelPtr& camera_intrinsic,
    const Eigen::Matrix4d& world2camera_pose,
    std::vector<Eigen::Vector2d>* radar_box2d_vertices) {
  const double camera_height = camera->GetBaseObject()->size(2);
  std::vector<Eigen::Vector3d> modified_radar_box_vertices = radar_box_vertices;
  for (size_t i = 0; i < 4; ++i) {
    modified_radar_box_vertices[i + 4].z() =
        modified_radar_box_vertices[i].z() + camera_height;
  }
  radar_box2d_vertices->reserve(radar_box_vertices.size());
  for (const auto& box_vertex_3d : modified_radar_box_vertices) {
    Eigen::Vector4d local_box_vertex =
        world2camera_pose * box_vertex_3d.homogeneous();
    Eigen::Vector2f temp_vertex =
        camera_intrinsic->Project(local_box_vertex.head(3).cast<float>());
    radar_box2d_vertices->push_back(temp_vertex.cast<double>());
  }
}

base::BaseCameraModelPtr TrackObjectDistance::QueryCameraModel(
    const SensorObjectConstPtr& camera) {
  return SensorDataManager::Instance()->GetCameraIntrinsic(
      camera->GetSensorId());
}

bool TrackObjectDistance::QueryWorld2CameraPose(
    const SensorObjectConstPtr& camera, Eigen::Matrix4d* pose) {
  Eigen::Affine3d camera2world_pose;
  bool status = SensorDataManager::Instance()->GetPose(
      camera->GetSensorId(), camera->GetTimestamp(), &camera2world_pose);
  if (!status) {
    return false;
  }
  (*pose) = camera2world_pose.matrix().inverse();
  return true;
}

bool TrackObjectDistance::QueryLidar2WorldPose(
    const SensorObjectConstPtr& lidar, Eigen::Matrix4d* pose) {
  Eigen::Affine3d velo2world_pose;
  if (!lidar->GetRelatedFramePose(&velo2world_pose)) {
    return false;
  }
  (*pose) = velo2world_pose.matrix();
  return true;
}

ProjectionCacheObject* TrackObjectDistance::BuildProjectionCacheObject(
    const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera,
    const base::BaseCameraModelPtr& camera_model,
    const std::string& measurement_sensor_id, double measurement_timestamp,
    const std::string& projection_sensor_id, double projection_timestamp) {
  // 1. get lidar2camera_pose
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return nullptr;
  }
  Eigen::Matrix4d lidar2world_pose;
  if (!QueryLidar2WorldPose(lidar, &lidar2world_pose)) {
    return nullptr;
  }
  Eigen::Matrix4d lidar2camera_pose =
      static_cast<Eigen::Matrix<double, 4, 4, 0, 4, 4>>(world2camera_pose *
                                                        lidar2world_pose);
  // 2. compute offset
  double time_diff = camera->GetTimestamp() - lidar->GetTimestamp();
  Eigen::Vector3d offset =
      lidar->GetBaseObject()->velocity.cast<double>() * time_diff;
  // 3. build projection cache
  const base::PointFCloud& cloud =
      lidar->GetBaseObject()->lidar_supplement.cloud;
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  const int lidar_object_id = lidar->GetBaseObject()->id;
  ProjectionCacheObject* cache_object = projection_cache_.BuildObject(
      measurement_sensor_id, measurement_timestamp, projection_sensor_id,
      projection_timestamp, lidar_object_id);
  if (cache_object == nullptr) {
    AERROR << "Failed to build projection cache object";
    return nullptr;
  }
  size_t start_ind = projection_cache_.GetPoint2dsSize();
  size_t end_ind = projection_cache_.GetPoint2dsSize();
  float xmin = FLT_MAX;
  float ymin = FLT_MAX;
  float xmax = -FLT_MAX;
  float ymax = -FLT_MAX;
  // 4. check whether all lidar's 8 3d vertices would projected outside frustum,
  // if not, build projection object of its cloud and cache it
  // else, build empty projection object and cache it
  bool is_all_lidar_3d_vertices_outside_frustum = false;
  if (cloud.size() > s_lidar2camera_projection_vertices_check_pts_num_) {
    is_all_lidar_3d_vertices_outside_frustum = true;
    std::vector<Eigen::Vector3d> lidar_box_vertices;
    GetObjectEightVertices(lidar->GetBaseObject(), &lidar_box_vertices);
    for (size_t i = 0; i < lidar_box_vertices.size(); ++i) {
      Eigen::Vector3d& vt = lidar_box_vertices[i];
      Eigen::Vector4d project_vt =
          static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
              world2camera_pose * Eigen::Vector4d(vt(0) + offset(0),
                                                  vt(1) + offset(1),
                                                  vt(2) + offset(2), 1.0));
      if (project_vt(2) <= 0) {
        continue;
      }
      Eigen::Vector2f project_vt2f = camera_model->Project(Eigen::Vector3f(
          static_cast<float>(project_vt(0)), static_cast<float>(project_vt(1)),
          static_cast<float>(project_vt(2))));
      if (!IsPtInFrustum(project_vt2f, width, height)) {
        continue;
      }
      is_all_lidar_3d_vertices_outside_frustum = false;
      break;
    }
  }
  // 5. if not all lidar 3d vertices outside frustum, build projection object
  // of its cloud and cache it, else build & cache an empty one.
  if (!is_all_lidar_3d_vertices_outside_frustum) {
    // 5.1 check whehter downsampling needed
    size_t every_n = 1;
    if (cloud.size() > s_lidar2camera_projection_downsample_target_pts_num_) {
      every_n =
          cloud.size() / s_lidar2camera_projection_downsample_target_pts_num_;
    }
    for (size_t i = 0; i < cloud.size(); ++i) {
      if ((every_n > 1) && (i % every_n != 0)) {
        continue;
      }
      const base::PointF& pt = cloud.at(i);
      Eigen::Vector4d project_pt =
          static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
              lidar2camera_pose * Eigen::Vector4d(pt.x + offset(0),
                                                  pt.y + offset(1),
                                                  pt.z + offset(2), 1.0));
      if (project_pt(2) <= 0) {
        continue;
      }
      Eigen::Vector2f project_pt2f = camera_model->Project(Eigen::Vector3f(
          static_cast<float>(project_pt(0)), static_cast<float>(project_pt(1)),
          static_cast<float>(project_pt(2))));
      if (!IsPtInFrustum(project_pt2f, width, height)) {
        continue;
      }
      if (project_pt2f.x() < xmin) {
        xmin = project_pt2f.x();
      }
      if (project_pt2f.y() < ymin) {
        ymin = project_pt2f.y();
      }
      if (project_pt2f.x() > xmax) {
        xmax = project_pt2f.x();
      }
      if (project_pt2f.y() > ymax) {
        ymax = project_pt2f.y();
      }
      projection_cache_.AddPoint(project_pt2f);
    }
  }
  end_ind = projection_cache_.GetPoint2dsSize();
  cache_object->SetStartInd(start_ind);
  cache_object->SetEndInd(end_ind);
  base::BBox2DF box = base::BBox2DF(xmin, ymin, xmax, ymax);
  cache_object->SetBox(box);
  return cache_object;
}

ProjectionCacheObject* TrackObjectDistance::QueryProjectionCacheObject(
    const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera,
    const base::BaseCameraModelPtr& camera_model,
    const bool measurement_is_lidar) {
  // 1. try to query existed projection cache object
  const std::string& measurement_sensor_id =
      measurement_is_lidar ? lidar->GetSensorId() : camera->GetSensorId();
  const double measurement_timestamp =
      measurement_is_lidar ? lidar->GetTimestamp() : camera->GetTimestamp();
  const std::string& projection_sensor_id =
      measurement_is_lidar ? camera->GetSensorId() : lidar->GetSensorId();
  const double projection_timestamp =
      measurement_is_lidar ? camera->GetTimestamp() : lidar->GetTimestamp();
  const int lidar_object_id = lidar->GetBaseObject()->id;
  ProjectionCacheObject* cache_object = projection_cache_.QueryObject(
      measurement_sensor_id, measurement_timestamp, projection_sensor_id,
      projection_timestamp, lidar_object_id);
  if (cache_object != nullptr) {
    return cache_object;
  }  // 2. if query failed, build projection and cache it
  return BuildProjectionCacheObject(
      lidar, camera, camera_model, measurement_sensor_id, measurement_timestamp,
      projection_sensor_id, projection_timestamp);
}

void TrackObjectDistance::QueryProjectedVeloCtOnCamera(
    const SensorObjectConstPtr& velodyne64, const SensorObjectConstPtr& camera,
    const Eigen::Matrix4d& lidar2camera_pose, Eigen::Vector3d* projected_ct) {
  double time_diff = camera->GetTimestamp() - velodyne64->GetTimestamp();
  Eigen::Vector3d offset =
      velodyne64->GetBaseObject()->velocity.cast<double>() * time_diff;
  const Eigen::Vector3d& velo_ct = velodyne64->GetBaseObject()->center;
  Eigen::Vector4d projected_ct_4d =
      static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
          lidar2camera_pose * Eigen::Vector4d(velo_ct[0] + offset[0],
                                              velo_ct[1] + offset[1],
                                              velo_ct[2] + offset[2], 1.0));
  *projected_ct = projected_ct_4d.head(3);
}

bool TrackObjectDistance::QueryPolygonDCenter(
    const base::ObjectConstPtr& object, const Eigen::Vector3d& ref_pos,
    const int range, Eigen::Vector3d* polygon_ct) {
  if (object == nullptr) {
    return false;
  }
  const base::PolygonDType& polygon = object->polygon;
  if (!ComputePolygonCenter(polygon, ref_pos, range, polygon_ct)) {
    return false;
  }
  return true;
}

bool TrackObjectDistance::IsTrackIdConsistent(
    const SensorObjectConstPtr& object1, const SensorObjectConstPtr& object2) {
  if (object1 == nullptr || object2 == nullptr) {
    return false;
  }
  if (object1->GetBaseObject()->track_id ==
      object2->GetBaseObject()->track_id) {
    return true;
  }
  return false;
}

bool TrackObjectDistance::LidarCameraCenterDistanceExceedDynamicThreshold(
    const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera) {
  double center_distance =
      (lidar->GetBaseObject()->center - camera->GetBaseObject()->center)
          .head(2)
          .norm();
  double local_distance = 60;
  const base::PointFCloud& cloud =
      lidar->GetBaseObject()->lidar_supplement.cloud;
  if (cloud.size() > 0) {
    const base::PointF& pt = cloud.at(0);
    local_distance = std::sqrt(pt.x * pt.x + pt.y * pt.y);
  }
  double dynamic_threshold = 5 + 0.15 * local_distance;
  if (center_distance > dynamic_threshold) {
    return true;
  }
  return false;
}

// @brief: compute the distance between input fused track and sensor object
// @return track object distance
float TrackObjectDistance::Compute(const TrackPtr& fused_track,
                                   const SensorObjectPtr& sensor_object,
                                   const TrackObjectDistanceOptions& options) {
  FusedObjectPtr fused_object = fused_track->GetFusedObject();
  if (fused_object == nullptr) {
    AERROR << "fused object is nullptr";
    return (std::numeric_limits<float>::max)();
  }
  Eigen::Vector3d* ref_point = options.ref_point;
  if (ref_point == nullptr) {
    AERROR << "reference point is nullptr";
    return (std::numeric_limits<float>::max)();
  }
  float distance = (std::numeric_limits<float>::max)();
  float min_distance = (std::numeric_limits<float>::max)();
  SensorObjectConstPtr lidar_object = fused_track->GetLatestLidarObject();
  SensorObjectConstPtr radar_object = fused_track->GetLatestRadarObject();
  SensorObjectConstPtr camera_object = fused_track->GetLatestCameraObject();
  if (IsLidar(sensor_object)) {
    if (lidar_object != nullptr) {
      distance = ComputeLidarLidar(lidar_object, sensor_object, *ref_point);
      min_distance = std::min(distance, min_distance);
    }
    if (radar_object != nullptr) {
      distance = ComputeLidarRadar(radar_object, sensor_object, *ref_point);
      min_distance = std::min(distance, min_distance);
    }
    if (camera_object != nullptr) {
      bool is_lidar_track_id_consistent =
          IsTrackIdConsistent(lidar_object, sensor_object);
      distance = ComputeLidarCamera(sensor_object, camera_object, true,
                                    is_lidar_track_id_consistent);
      min_distance = std::min(distance, min_distance);
    }
  } else if (IsRadar(sensor_object)) {
    if (lidar_object != nullptr) {
      distance = ComputeLidarRadar(lidar_object, sensor_object, *ref_point);
      min_distance = std::min(distance, min_distance);
    }
    // else if (radar_object != nullptr) {
    //   distance = std::numeric_limits<float>::max();
    //   min_distance = std::min(distance, min_distance);
    // }
    if (camera_object != nullptr) {
      distance = ComputeRadarCamera(sensor_object, camera_object);
      min_distance = std::min(distance, min_distance);
    }
  } else if (IsCamera(sensor_object)) {
    if (lidar_object != nullptr) {
      bool is_camera_track_id_consistent =
          IsTrackIdConsistent(camera_object, sensor_object);
      distance = ComputeLidarCamera(lidar_object, sensor_object, false,
                                    is_camera_track_id_consistent);
      min_distance = std::min(distance, min_distance);
    }

  } else {
    AERROR << "fused sensor type is not support";
  }
  return min_distance;
}

// @brief: compute the distance between velodyne64 observation and
// velodyne64 observation
// @return the distance of velodyne64 vs. velodyne64
float TrackObjectDistance::ComputeLidarLidar(
    const SensorObjectConstPtr& fused_object,
    const SensorObjectPtr& sensor_object, const Eigen::Vector3d& ref_pos,
    int range) {
  double center_distance = (sensor_object->GetBaseObject()->center -
                            fused_object->GetBaseObject()->center)
                               .head(2)
                               .norm();
  if (center_distance > s_lidar2lidar_association_center_dist_threshold_) {
    ADEBUG << "center distance exceed lidar2lidar tight threshold: "
           << "center_dist@" << center_distance << ", "
           << "tight_threh@"
           << s_lidar2lidar_association_center_dist_threshold_;
    return (std::numeric_limits<float>::max)();
  }
  float distance =
      ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "ComputeLidarLidar distance: " << distance;
  return distance;
}

// @brief: compute the distance between velodyne64 observation and
// radar observation
// @return distance of velodyne64 vs. radar
float TrackObjectDistance::ComputeLidarRadar(
    const SensorObjectConstPtr& fused_object,
    const SensorObjectPtr& sensor_object, const Eigen::Vector3d& ref_pos,
    int range) {
  double center_distance = (sensor_object->GetBaseObject()->center -
                            fused_object->GetBaseObject()->center)
                               .head(2)
                               .norm();
  if (center_distance > s_lidar2radar_association_center_dist_threshold_) {
    ADEBUG << "center distance exceed lidar2radar tight threshold: "
           << "center_dist@" << center_distance << ", "
           << "tight_threh@"
           << s_lidar2radar_association_center_dist_threshold_;
    return (std::numeric_limits<float>::max)();
  }
  float distance =
      ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "ComputeLidarRadar distance: " << distance;
  return distance;
}

// @brief: compute the distance between radar observation and
// radar observation
// @return distance of radar vs. radar
float TrackObjectDistance::ComputeRadarRadar(
    const SensorObjectPtr& fused_object, const SensorObjectPtr& sensor_object,
    const Eigen::Vector3d& ref_pos, int range) {
  double center_distance = (sensor_object->GetBaseObject()->center -
                            fused_object->GetBaseObject()->center)
                               .head(2)
                               .norm();
  if (center_distance > s_radar2radar_association_center_dist_threshold_) {
    ADEBUG << "center distance exceed radar2radar tight threshold: "
           << "center_dist@" << center_distance << ", "
           << "tight_threh@"
           << s_radar2radar_association_center_dist_threshold_;
    return (std::numeric_limits<float>::max)();
  }
  float distance =
      ComputePolygonDistance3d(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "ComputeRadarRadar distance: " << distance;
  return distance;
}

// @brief: compute the distance between lidar observation and
// camera observation
// @return distance of lidar vs. camera
float TrackObjectDistance::ComputeLidarCamera(
    const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera,
    const bool measurement_is_lidar, const bool is_track_id_consistent) {
  if (!is_track_id_consistent) {
    if (LidarCameraCenterDistanceExceedDynamicThreshold(lidar, camera)) {
      return distance_thresh_;
    }
  }
  float distance = distance_thresh_;
  // 1. get camera intrinsic and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    AERROR << "Failed to get camera model for " << camera->GetSensorId();
    return distance;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    AERROR << "Failed to query camera pose";
    return distance;
  }
  Eigen::Matrix4d lidar2world_pose;
  if (!QueryLidar2WorldPose(lidar, &lidar2world_pose)) {
    AERROR << "Failed to query lidar pose";
    return distance;
  }
  Eigen::Matrix4d lidar2camera_pose =
      static_cast<Eigen::Matrix<double, 4, 4, 0, 4, 4>>(world2camera_pose *
                                                        lidar2world_pose);
  // 2. compute distance of camera vs. lidar observation
  const base::PointFCloud& cloud =
      lidar->GetBaseObject()->lidar_supplement.cloud;
  const base::BBox2DF& camera_bbox =
      camera->GetBaseObject()->camera_supplement.box;
  const base::Point2DF camera_bbox_ct = camera_bbox.Center();
  const Eigen::Vector2d box2d_ct =
      Eigen::Vector2d(camera_bbox_ct.x, camera_bbox_ct.y);
  ADEBUG << "object cloud size : " << cloud.size();
  if (cloud.size() > 0) {
    // 2.1 if cloud is not empty, calculate distance according to pts box
    // similarity
    ProjectionCacheObject* cache_object = QueryProjectionCacheObject(
        lidar, camera, camera_model, measurement_is_lidar);
    if (cache_object == nullptr) {
      AERROR << "Failed to query projection cached object";
      return distance;
    }
    double similarity =
        ComputePtsBoxSimilarity(&projection_cache_, cache_object, camera_bbox);
    distance =
        distance_thresh_ * ((1.0f - static_cast<float>(similarity)) /
                            (1.0f - vc_similarity2distance_penalize_thresh_));
  } else {
    // 2.2 if cloud is empty, calculate distance according to ct diff
    Eigen::Vector3d projected_velo_ct;
    QueryProjectedVeloCtOnCamera(lidar, camera, lidar2camera_pose,
                                 &projected_velo_ct);
    if (projected_velo_ct[2] > 0) {
      Eigen::Vector2f project_pt2f = camera_model->Project(
          Eigen::Vector3f(static_cast<float>(projected_velo_ct[0]),
                          static_cast<float>(projected_velo_ct[1]),
                          static_cast<float>(projected_velo_ct[2])));
      Eigen::Vector2d project_pt2d = project_pt2f.cast<double>();
      Eigen::Vector2d ct_diff = project_pt2d - box2d_ct;
      distance =
          static_cast<float>(ct_diff.norm()) * vc_diff2distance_scale_factor_;
    } else {
      distance = std::numeric_limits<float>::max();
    }
  }
  ADEBUG << "ComputeLidarCamera distance: " << distance;
  return distance;
}

// @brief: compute the distance between radar observation and
// camera observation
// @return distance of radar vs. camera
float TrackObjectDistance::ComputeRadarCamera(
    const SensorObjectConstPtr& radar, const SensorObjectConstPtr& camera) {
  float distance = distance_thresh_;
  // 1. get camera model and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    AERROR << "Failed to get camera model for " << camera->GetSensorId();
    return distance;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return distance;
  }
  // get camera useful information
  const base::BBox2DF& camera_bbox =
      camera->GetBaseObject()->camera_supplement.box;
  const base::Point2DF camera_bbox_ct = camera_bbox.Center();
  const Eigen::Vector2d box2d_ct =
      Eigen::Vector2d(camera_bbox_ct.x, camera_bbox_ct.y);
  Eigen::Vector2d box2d_size = Eigen::Vector2d(
      camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  box2d_size = box2d_size.cwiseMax(rc_min_box_size_);
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  // get radar useful information
  double time_diff = camera->GetTimestamp() - radar->GetTimestamp();
  Eigen::Vector3d offset =
      radar->GetBaseObject()->velocity.cast<double>() * time_diff;
  offset.setZero();
  Eigen::Vector3d radar_ct = radar->GetBaseObject()->center + offset;
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose *
      Eigen::Vector4d(radar_ct[0], radar_ct[1], radar_ct[2], 1.0));
  std::vector<Eigen::Vector3d> radar_box_vertices;
  GetObjectEightVertices(radar->GetBaseObject(), &radar_box_vertices);
  std::vector<Eigen::Vector2d> radar_box2d_vertices;
  GetModified2DRadarBoxVertices(radar_box_vertices, camera, camera_model,
                                world2camera_pose, &radar_box2d_vertices);
  // compute similarity
  double fused_similarity = 0.0;
  if (local_pt[2] > 0) {
    Eigen::Vector3f pt3f;
    pt3f << camera_model->Project(Eigen::Vector3f(
        static_cast<float>(local_pt[0]), static_cast<float>(local_pt[1]),
        static_cast<float>(local_pt[2]))),
        static_cast<float>(local_pt[2]);
    Eigen::Vector3d pt3d = pt3f.cast<double>();
    if (IsPtInFrustum(pt3d, width, height)) {
      // compute similarity on x direction
      double x_similarity = ComputeRadarCameraXSimilarity(
          pt3d.x(), box2d_ct.x(), box2d_size.x(), rc_x_similarity_params_);
      // compute similarity on y direction
      double y_similarity = ComputeRadarCameraYSimilarity(
          pt3d.y(), box2d_ct.y(), box2d_size.y(), rc_y_similarity_params_);
      // compute similarity on height
      // use camera car height to modify the radar location
      // double h_similarity = ComputeRadarCameraHSimilarity(
      //     radar, camera, box2d_size.y(), radar_box2d_vertices,
      //     rc_h_similarity_params_);
      // compute similarity on width
      // double w_similarity = ComputeRadarCameraWSimilarity(
      //     radar, width, box2d_size.x(), radar_box2d_vertices,
      //     rc_w_similarity_params_);
      // compute similarity on offset 3d
      double loc_similarity = ComputeRadarCameraLocSimilarity(
          radar_ct, camera, world2camera_pose, rc_loc_similarity_params_);
      // compute similarity on velocity
      double velocity_similarity =
          ComputeRadarCameraVelocitySimilarity(radar, camera);
      // fuse similarity
      std::vector<double> multiple_similarities = {
          x_similarity, y_similarity, loc_similarity, velocity_similarity
          // height_similarity, width_similarity,
      };
      fused_similarity = FuseMultipleProbabilities(multiple_similarities);
    }
  }
  distance = distance_thresh_ * static_cast<float>(1.0 - fused_similarity) /
             (1.0f - rc_similarity2distance_penalize_thresh_);
  ADEBUG << "ComputeRadarCamera distance: " << distance;
  return distance;
}

// @brief: compute the distance between camera observation and
// camera observation
// @return the distance of camera vs. camera
float TrackObjectDistance::ComputeCameraCamera(
    const SensorObjectPtr& fused_camera, const SensorObjectPtr& sensor_camera) {
  return (std::numeric_limits<float>::max());
}

// @brief: calculate the similarity between velodyne64 observation and
// camera observation
// @return the similarity which belongs to [0, 1]. When velodyne64
// observation is similar to the camera one, the similarity would
// close to 1. Otherwise, it would close to 0.
// @key idea:
// 1. get camera intrinsic and pose
// 2. compute similarity between camera's box and velodyne64's pts
// within camera's frustum
// @NOTE: original method name is compute_velodyne64_camera_dist_score
double TrackObjectDistance::ComputeLidarCameraSimilarity(
    const SensorObjectConstPtr& lidar, const SensorObjectConstPtr& camera,
    const bool measurement_is_lidar) {
  double similarity = 0.0;
  // 1. get camera intrinsic and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    AERROR << "Failed to get camera model for " << camera->GetSensorId();
    return similarity;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return similarity;
  }
  Eigen::Matrix4d lidar2world_pose;
  if (!QueryLidar2WorldPose(lidar, &lidar2world_pose)) {
    return similarity;
  }
  // Eigen::Matrix4d lidar2camera_pose = world2camera_pose * lidar2world_pose;
  // 2. compute similarity of camera vs. velodyne64 observation
  const base::PointFCloud& cloud =
      lidar->GetBaseObject()->lidar_supplement.cloud;
  const base::BBox2DF& camera_bbox =
      camera->GetBaseObject()->camera_supplement.box;
  if (cloud.size() > 0) {
    ProjectionCacheObject* cache_object = QueryProjectionCacheObject(
        lidar, camera, camera_model, measurement_is_lidar);
    if (cache_object == nullptr) {
      return similarity;
    }
    similarity =
        ComputePtsBoxSimilarity(&projection_cache_, cache_object, camera_bbox);
  }
  return similarity;
}

// @brief: calculate the similarity between radar observation and
// camera observation
// @return the similarity which belongs to [0, 1]. When radar
// observation is similar to the camera one, the similarity would
// close to 1. Otherwise, it would close to 0.
// @NOTE: original method name is compute_radar_camera_dist_score
// @TODO: THIS METHOD SHOULD RETURN 0, IF RADAR IS IN FRONT OF CAMERA DETECTION
double TrackObjectDistance::ComputeRadarCameraSimilarity(
    const SensorObjectConstPtr& radar, const SensorObjectConstPtr& camera) {
  double similarity = 0.0;
  // 1. get camera intrinsic and pose
  base::BaseCameraModelPtr camera_model = QueryCameraModel(camera);
  if (camera_model == nullptr) {
    AERROR << "Failed to get camera model for " << camera->GetSensorId();
    return similarity;
  }
  Eigen::Matrix4d world2camera_pose;
  if (!QueryWorld2CameraPose(camera, &world2camera_pose)) {
    return similarity;
  }
  // 2. get information of camera
  const base::BBox2DF& camera_bbox =
      camera->GetBaseObject()->camera_supplement.box;
  const base::Point2DF camera_bbox_ct = camera_bbox.Center();
  const Eigen::Vector2d box2d_ct =
      Eigen::Vector2d(camera_bbox_ct.x, camera_bbox_ct.y);
  Eigen::Vector2d box2d_size = Eigen::Vector2d(
      camera_bbox.xmax - camera_bbox.xmin, camera_bbox.ymax - camera_bbox.ymin);
  box2d_size = box2d_size.cwiseMax(rc_min_box_size_);
  double width = static_cast<double>(camera_model->get_width());
  double height = static_cast<double>(camera_model->get_height());
  // 3. get information of radar
  Eigen::Vector3d radar_ct = radar->GetBaseObject()->center;
  Eigen::Vector4d local_pt = static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
      world2camera_pose *
      Eigen::Vector4d(radar_ct[0], radar_ct[1], radar_ct[2], 1.0));
  // 4. similarity computation
  Eigen::Vector3d camera_ct = camera->GetBaseObject()->center;
  Eigen::Vector3d camera_ct_c =
      (world2camera_pose * camera_ct.homogeneous()).head(3);
  double depth_diff = local_pt.z() - camera_ct_c.z();
  const double depth_diff_th = 0.1;
  depth_diff /= camera_ct_c.z();
  if (local_pt[2] > 0 && depth_diff > -depth_diff_th) {
    Eigen::Vector3f pt3f;
    pt3f << camera_model->Project(Eigen::Vector3f(
        static_cast<float>(local_pt[0]), static_cast<float>(local_pt[1]),
        static_cast<float>(local_pt[2]))),
        static_cast<float>(local_pt[2]);
    Eigen::Vector3d pt3d = pt3f.cast<double>();
    if (IsPtInFrustum(pt3d, width, height)) {
      rc_x_similarity_params_2_.welsh_loss_scale_ =
          rc_x_similarity_params_2_welsh_loss_scale_;
      // compute similarity on x direction
      double x_similarity = ComputeRadarCameraXSimilarity(
          pt3d.x(), box2d_ct.x(), box2d_size.x(), rc_x_similarity_params_2_);
      // compute similarity on y direction
      double y_similarity = ComputeRadarCameraXSimilarity(
          pt3d.y(), box2d_ct.y(), box2d_size.y(), rc_x_similarity_params_2_);
      std::vector<double> multiple_similarities = {x_similarity, y_similarity};
      similarity = FuseMultipleProbabilities(multiple_similarities);
    }
  }
  return similarity;
}

// @brief: compute polygon distance between fused object and sensor object
// @return 3d distance between fused object and sensor object
float TrackObjectDistance::ComputePolygonDistance3d(
    const SensorObjectConstPtr& fused_object,
    const SensorObjectPtr& sensor_object, const Eigen::Vector3d& ref_pos,
    int range) {
  const base::ObjectConstPtr& obj_f = fused_object->GetBaseObject();
  Eigen::Vector3d fused_poly_center(0, 0, 0);
  if (!QueryPolygonDCenter(obj_f, ref_pos, range, &fused_poly_center)) {
    return (std::numeric_limits<float>::max());
  }
  const base::ObjectConstPtr obj_s = sensor_object->GetBaseObject();
  Eigen::Vector3d sensor_poly_center(0, 0, 0);
  if (!QueryPolygonDCenter(obj_s, ref_pos, range, &sensor_poly_center)) {
    return (std::numeric_limits<float>::max());
  }
  double fusion_timestamp = fused_object->GetTimestamp();
  double sensor_timestamp = sensor_object->GetTimestamp();
  double time_diff = sensor_timestamp - fusion_timestamp;
  fused_poly_center(0) += obj_f->velocity(0) * time_diff;
  fused_poly_center(1) += obj_f->velocity(1) * time_diff;
  float distance =
      ComputeEuclideanDistance(fused_poly_center, sensor_poly_center);
  return distance;
}

// @brief: compute euclidean distance of input pts
// @return eculidean distance of input pts
float TrackObjectDistance::ComputeEuclideanDistance(
    const Eigen::Vector3d& des, const Eigen::Vector3d& src) {
  Eigen::Vector3d diff_pos = des - src;
  float distance = static_cast<float>(
      std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum()));
  return distance;
}

// @brief: compute polygon center
// @return true if get center successfully, otherwise return false
bool TrackObjectDistance::ComputePolygonCenter(
    const base::PolygonDType& polygon, Eigen::Vector3d* center) {
  int size = static_cast<int>(polygon.size());
  if (size == 0) {
    return false;
  }
  *center = Eigen::Vector3d(0, 0, 0);
  for (int i = 0; i < size; ++i) {
    const auto& point = polygon.at(i);
    (*center)[0] += point.x;
    (*center)[1] += point.y;
  }
  (*center) /= size;
  return true;
}

// @brief: compute polygon center
// @return true if get center successfully, otherwise return false
bool TrackObjectDistance::ComputePolygonCenter(
    const base::PolygonDType& polygon, const Eigen::Vector3d& ref_pos,
    int range, Eigen::Vector3d* center) {
  base::PolygonDType polygon_part;
  std::map<double, int> distance2idx;
  for (size_t idx = 0; idx < polygon.size(); ++idx) {
    const auto& point = polygon.at(idx);
    double distance =
        sqrt(pow(point.x - ref_pos(0), 2) + pow(point.y - ref_pos(1), 2));
    distance2idx.insert(std::make_pair(distance, idx));
  }
  int size = static_cast<int>(distance2idx.size());
  int nu = std::max(range, size / range + 1);
  nu = std::min(nu, size);
  int count = 0;
  std::map<double, int>::iterator it = distance2idx.begin();
  for (; it != distance2idx.end(), count < nu; ++it, ++count) {
    polygon_part.push_back(polygon[it->second]);
  }
  bool state = ComputePolygonCenter(polygon_part, center);
  return state;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
