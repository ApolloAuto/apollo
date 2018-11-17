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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track_object_distance.h"

#include <algorithm>
#include <limits>
#include <set>
#include <utility>

#include "Eigen/StdVector"
#include "boost/format.hpp"

#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d);

DECLARE_bool(async_fusion);
DECLARE_bool(use_distance_angle_fusion);

namespace apollo {
namespace perception {

float PbfTrackObjectDistance::Compute(
    PbfTrackPtr fused_track,
    const std::shared_ptr<PbfSensorObject> &sensor_object,
    const TrackObjectDistanceOptions &options) {
  const SensorType &sensor_type = sensor_object->sensor_type;
  ADEBUG << "sensor type: " << static_cast<int>(sensor_type);
  std::shared_ptr<PbfSensorObject> fused_object = fused_track->GetFusedObject();
  if (fused_object == nullptr) {
    ADEBUG << "fused object is nullptr";
    return std::numeric_limits<float>::max();
  }

  Eigen::Vector3d *ref_point = options.ref_point;
  if (ref_point == nullptr) {
    AERROR << "reference point is nullptr";
    return std::numeric_limits<float>::max();
  }

  float distance = std::numeric_limits<float>::max();
  const std::shared_ptr<PbfSensorObject> &lidar_object =
      fused_track->GetLatestLidarObject();
  const std::shared_ptr<PbfSensorObject> &radar_object =
      fused_track->GetLatestRadarObject();

  if (FLAGS_use_distance_angle_fusion) {
    AINFO << "use distance angle fusion";
    distance =
        ComputeDistanceAngleMatchProb(fused_object, sensor_object, options);
  } else {
    AINFO << "use traditional lidar camera radar fusion";

    if (is_lidar(sensor_type)) {
      if (lidar_object != nullptr) {
        distance = ComputeVelodyne64Velodyne64(fused_object, sensor_object,
                                               *ref_point);
      } else if (radar_object != nullptr) {
        distance =
            ComputeVelodyne64Radar(sensor_object, fused_object, *ref_point);
      } else {
        AWARN << "All of the objects are nullptr";
      }
    } else if (is_radar(sensor_type)) {
      if (lidar_object != nullptr) {
        distance =
            ComputeVelodyne64Radar(fused_object, sensor_object, *ref_point);
      } else if (radar_object != nullptr) {
        distance = std::numeric_limits<float>::max();
        //    distance = compute_radar_radar(fused_object, sensor_object,
        //    *ref_point);
      } else {
        AWARN << "All of the objects are nullptr";
      }
    } else {
      AERROR << "fused sensor type is not support";
    }
  }
  return distance;
}

float PbfTrackObjectDistance::ComputeVelodyne64Velodyne64(
    const std::shared_ptr<PbfSensorObject> &fused_object,
    const std::shared_ptr<PbfSensorObject> &sensor_object,
    const Eigen::Vector3d &ref_pos, int range) {
  float distance =
      ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "compute_velodyne64_velodyne64 distance: " << distance;
  return distance;
}

float PbfTrackObjectDistance::ComputeVelodyne64Radar(
    const std::shared_ptr<PbfSensorObject> &fused_object,
    const std::shared_ptr<PbfSensorObject> &sensor_object,
    const Eigen::Vector3d &ref_pos, int range) {
  float distance =
      ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "compute_velodyne64_radar distance " << distance;
  return distance;
}

float PbfTrackObjectDistance::ComputeRadarRadar(
    const std::shared_ptr<PbfSensorObject> &fused_object,
    const std::shared_ptr<PbfSensorObject> &sensor_object,
    const Eigen::Vector3d &ref_pos, int range) {
  float distance =
      ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "compute_radar_radar distance " << distance;
  return distance;
}

float PbfTrackObjectDistance::GetAngle(const Eigen::Vector3d &sensor_center,
                                       SensorType type) {
  Eigen::Vector3d center = sensor_center;

  if (!FLAGS_use_navigation_mode) {
    if (is_camera(type)) {
      center[0] = sensor_center[2];
      center[1] = sensor_center[0];
    } else if (is_lidar(type)) {
      center[0] = sensor_center[0];
      center[1] = -sensor_center[1];
    }
  }

  if (center[0] == 0) {
    if (center[1] > 0) {
      return M_PI / 2;
    } else {
      return -M_PI / 2;
    }
  }

  return std::atan2(center[1], center[0]);
}

Eigen::Vector3d PbfTrackObjectDistance::GetCenter(
    const std::shared_ptr<PbfSensorObject> &obj,
    const Eigen::Matrix4d &world_sensor_pose, SensorType sensor_type) {
  if (!FLAGS_use_navigation_mode) {
    Eigen::Vector3d sensor_center =
        (world_sensor_pose *
         Eigen::Vector4d(obj->object->center[0], obj->object->center[1], 0, 1))
            .head(3);
    // normalize to rfu
    Eigen::Vector3d center = sensor_center;
    if (is_camera(sensor_type)) {
      center[0] = sensor_center[0];
      center[1] = sensor_center[2];
    } else if (is_lidar(sensor_type)) {
      center[0] = -sensor_center[1];
      center[1] = sensor_center[0];
    }
    return center;
  }
  return obj->object->center;
}

float PbfTrackObjectDistance::ComputeDistanceAngleMatchProb(
    const std::shared_ptr<PbfSensorObject> &fused_object,
    const std::shared_ptr<PbfSensorObject> &sensor_object,
    const TrackObjectDistanceOptions &options) {
  static float weight_x = 0.8f;
  static float weight_y = 0.2f;
  static float speed_diff = 5.0f;
  static float epislon = 0.1f;
  static float angle_tolerance = 5.0f;
  static float distance_tolerance_max = 5.0f;
  static float distance_tolerance_min = 2.0f;

  const std::shared_ptr<Object> &fobj = fused_object->object;
  const std::shared_ptr<Object> &sobj = sensor_object->object;

  if (fobj == nullptr || sobj == nullptr) {
    AERROR << "Object is nullptr.";
    return std::numeric_limits<float>::max();
  }

  const Eigen::Matrix4d &world_sensor_pose =
      options.sensor_world_pose->inverse();

  Eigen::Vector3d fcenter =
      GetCenter(fused_object, world_sensor_pose, sensor_object->sensor_type);
  Eigen::Vector3d scenter =
      GetCenter(sensor_object, world_sensor_pose, sensor_object->sensor_type);

  AINFO << "fcenter is " << fcenter;
  AINFO << "scenter is " << scenter;

  float euclid_dist =
      static_cast<float>(((fcenter.head(2) - scenter.head(2)).norm()));

  if (is_camera(sensor_object->sensor_type)) {
    AINFO << " camera object sensor before check for distance with id"
          << sensor_object->object->track_id << " with dist " << euclid_dist
          << " with fused track id " << fused_object->object->track_id;
  }

  /*if (euclid_dist > distance_tolerance_max) {
    return std::numeric_limits<float>::max();
  }*/

  float range_distance_ratio = std::numeric_limits<float>::max();
  float angle_distance_diff = 0.0f;

  if (fcenter(0) > epislon && std::abs(fcenter(1)) > epislon) {
    float x_ratio = std::abs(fcenter(0) - scenter(0)) / fcenter(0);
    assert(x_ratio >= 0);
    float y_ratio = std::abs(fcenter(1) - scenter(1)) / std::abs(fcenter(1));

    if (x_ratio < FLAGS_pbf_fusion_assoc_distance_percent &&
        y_ratio < FLAGS_pbf_fusion_assoc_distance_percent) {
      range_distance_ratio = weight_x * x_ratio + weight_y * y_ratio;
    }

  } else if (fcenter(0) > epislon) {
    float x_ratio = std::abs(fcenter(0) - scenter(0)) / fcenter(0);
    if (x_ratio < FLAGS_pbf_fusion_assoc_distance_percent) {
      range_distance_ratio = x_ratio;
    }
  } else if (std::abs(fcenter(1)) > epislon) {
    float y_ratio = std::abs(fcenter(1) - scenter(1)) / std::abs(fcenter(1));
    if (y_ratio < FLAGS_pbf_fusion_assoc_distance_percent) {
      range_distance_ratio = y_ratio;
    }
  }
  float distance = range_distance_ratio;

  // if (is_radar(sensor_object->sensor_type)) {
  float sangle = GetAngle(scenter, sensor_object->sensor_type);
  float fangle = GetAngle(fcenter, sensor_object->sensor_type);
  angle_distance_diff = (std::abs(sangle - fangle) * 180) / M_PI;
  float fobject_dist = static_cast<float>(fcenter.norm());
  double svelocity = sobj->velocity.norm();
  double fvelocity = fobj->velocity.norm();
  if (svelocity > 0.0 && fvelocity > 0.0) {
    float cos_distance =
        sobj->velocity.dot(fobj->velocity) / (svelocity * fvelocity);
    if (cos_distance < FLAGS_pbf_distance_speed_cos_diff) {
      AWARN << "velocity cosine distance too large" << cos_distance;
      distance = std::numeric_limits<float>::max();
    }
  }

  if (std::abs(svelocity - fvelocity) > speed_diff ||
      angle_distance_diff > angle_tolerance) {
    AWARN << "angle " << angle_distance_diff << " tolerance " << angle_tolerance
          << " speed " << std::abs(svelocity - fvelocity) << " tolerance "
          << speed_diff;
    distance = std::numeric_limits<float>::max();
  }

  float distance_allowed =
      std::max(static_cast<float>(fobject_dist * sin(angle_distance_diff)),
               distance_tolerance_min);
  if (euclid_dist > distance_allowed) {
    AWARN << "angle distance too large  " << distance_allowed;
    distance = std::numeric_limits<float>::max();
  }
  AINFO << "distance calculated " << distance;
  //}
  return distance;
}

float PbfTrackObjectDistance::ComputeDistance3D(
    const std::shared_ptr<PbfSensorObject> &fused_object,
    const std::shared_ptr<PbfSensorObject> &sensor_object,
    const Eigen::Vector3d &ref_pos, const int range) {
  const std::shared_ptr<Object> &obj = fused_object->object;
  if (obj == nullptr) {
    AERROR << "Object is nullptr.";
    return std::numeric_limits<float>::max();
  }
  const PolygonDType &fused_poly = obj->polygon;
  Eigen::Vector3d fused_poly_center(0, 0, 0);
  bool fused_state =
      ComputePolygonCenter(fused_poly, ref_pos, range, &fused_poly_center);
  if (!fused_state) {
    AERROR << "fail to compute polygon center! fused polygon size:"
           << fused_poly.size();
    return std::numeric_limits<float>::max();
  }

  const std::shared_ptr<Object> obj2 = sensor_object->object;
  if (obj2 == nullptr) {
    AERROR << "Object is nullptr.";
    return std::numeric_limits<float>::max();
  }
  const PolygonDType &sensor_poly = obj2->polygon;
  Eigen::Vector3d sensor_poly_center(0, 0, 0);
  bool sensor_state =
      ComputePolygonCenter(sensor_poly, ref_pos, range, &sensor_poly_center);

  if (!sensor_state) {
    AERROR << "fail to compute sensor polygon center: polygon size:"
           << sensor_poly.size();
    return std::numeric_limits<float>::max();
  }

  const double time_diff = sensor_object->timestamp - fused_object->timestamp;
  fused_poly_center(0) += obj->velocity(0) * time_diff;
  fused_poly_center(1) += obj->velocity(1) * time_diff;
  return ComputeEuclideanDistance(fused_poly_center, sensor_poly_center);
}

float PbfTrackObjectDistance::ComputeEuclideanDistance(
    const Eigen::Vector3d &des, const Eigen::Vector3d &src) {
  Eigen::Vector3d diff_pos = des - src;
  return std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum());
}

bool PbfTrackObjectDistance::ComputePolygonCenter(const PolygonDType &polygon,
                                                  Eigen::Vector3d *center) {
  CHECK_NOTNULL(center);
  if (polygon.empty()) {
    return false;
  }
  *center = Eigen::Vector3d(0, 0, 0);
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto &point = polygon.points[i];
    (*center)(0) += point.x;
    (*center)(1) += point.y;
  }
  *center /= polygon.size();
  return true;
}

bool PbfTrackObjectDistance::ComputePolygonCenter(
    const PolygonDType &polygon, const Eigen::Vector3d &ref_pos, int range,
    Eigen::Vector3d *center) {
  CHECK_NOTNULL(center);
  PolygonDType polygon_part;
  std::set<std::pair<double, int>> distance2idx_set;

  for (size_t idx = 0; idx < polygon.size(); ++idx) {
    const auto &point = polygon.points[idx];
    double distance =
        sqrt(pow(point.x - ref_pos(0), 2) + pow(point.y - ref_pos(1), 2));
    distance2idx_set.insert(std::make_pair(distance, idx));
  }

  int size = distance2idx_set.size();
  int nu = std::min(size, std::max(range, size / range + 1));
  int count = 0;
  for (auto it = distance2idx_set.begin();
       it != distance2idx_set.end() && count < nu; ++it, ++count) {
    polygon_part.push_back(polygon[it->second]);
  }
  return ComputePolygonCenter(polygon_part, center);
}

}  // namespace perception
}  // namespace apollo
