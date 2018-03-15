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
#include <map>
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
    const PbfTrackPtr &fused_track, const PbfSensorObjectPtr &sensor_object,
    const TrackObjectDistanceOptions &options) {
  const SensorType &sensor_type = sensor_object->sensor_type;
  ADEBUG << "sensor type: " << static_cast<int>(sensor_type);
  PbfSensorObjectPtr fused_object = fused_track->GetFusedObject();
  if (fused_object == nullptr) {
    ADEBUG << "fused object is nullptr";
    return (std::numeric_limits<float>::max)();
  }

  Eigen::Vector3d *ref_point = options.ref_point;
  if (ref_point == nullptr) {
    AERROR << "reference point is nullptr";
    return (std::numeric_limits<float>::max)();
  }

  float distance = std::numeric_limits<float>::max();
  const PbfSensorObjectPtr &lidar_object = fused_track->GetLatestLidarObject();
  const PbfSensorObjectPtr &radar_object = fused_track->GetLatestRadarObject();

  if (!FLAGS_async_fusion) {
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
  } else {
    if (FLAGS_use_distance_angle_fusion) {
      distance = ComputeDistanceAngleMatchProb(fused_object, sensor_object);
    } else {
      AERROR << "other distance method not supported for async fusion";
    }
  }
  return distance;
}

float PbfTrackObjectDistance::ComputeVelodyne64Velodyne64(
    const PbfSensorObjectPtr &fused_object,
    const PbfSensorObjectPtr &sensor_object, const Eigen::Vector3d &ref_pos,
    int range) {
  float distance =
      ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "compute_velodyne64_velodyne64 distance: " << distance;
  return distance;
}

float PbfTrackObjectDistance::ComputeVelodyne64Radar(
    const PbfSensorObjectPtr &fused_object,
    const PbfSensorObjectPtr &sensor_object, const Eigen::Vector3d &ref_pos,
    int range) {
  float distance =
      ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "compute_velodyne64_radar distance " << distance;
  return distance;
}

float PbfTrackObjectDistance::ComputeRadarRadar(
    const PbfSensorObjectPtr &fused_object,
    const PbfSensorObjectPtr &sensor_object, const Eigen::Vector3d &ref_pos,
    int range) {
  float distance =
      ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
  ADEBUG << "compute_radar_radar distance " << distance;
  return distance;
}

float PbfTrackObjectDistance::GetAngle(const ObjectPtr &obj) {
  if (obj->center[0] == 0) {
    if (obj->center[1] > 0) {
      return M_PI / 2;
    } else {
      return -M_PI / 2;
    }
  }
  return std::atan2(obj->center[1], obj->center[0]);
}

float PbfTrackObjectDistance::ComputeDistanceAngleMatchProb(
    const PbfSensorObjectPtr &fused_object,
    const PbfSensorObjectPtr &sensor_object) {
  static float weight_x = 0.7;
  static float weight_y = 0.3;
  static float weight_range = 0.5;
  static float weight_angle = 0.5;
  static float angle_tolerance = 30;

  const ObjectPtr &fobj = fused_object->object;
  const ObjectPtr &sobj = sensor_object->object;

  if (fobj == nullptr || sobj == nullptr) {
    AERROR << "Object is nullptr.";
    return (std::numeric_limits<float>::max)();
  }

  Eigen::Vector3d &fcenter = fobj->center;
  Eigen::Vector3d &scenter = sobj->center;
  float range_distance_ratio = 0;
  float angle_distance_ratio = 0;

  if (fcenter[0] > 0 && fcenter[1] > 0) {
    range_distance_ratio =
        weight_x * std::abs(fcenter[0] - scenter[0]) / fcenter[0] +
        weight_y * std::abs(fcenter[1] - scenter[1]) / fcenter[1];
  } else if (fcenter[0] > 0) {
    range_distance_ratio = std::abs(fcenter[0] - scenter[0]) / fcenter[0];
  } else if (fcenter[1] > 0) {
    range_distance_ratio = std::abs(fcenter[1] - scenter[1]) / fcenter[1];
  }

  float sangle = GetAngle(sobj);
  float fangle = GetAngle(fobj);

  angle_distance_ratio =
      (std::abs(sangle - fangle) * 180) / (angle_tolerance * M_PI);

  float distance =
      weight_range * range_distance_ratio + weight_angle * angle_distance_ratio;

  return distance;
}

float PbfTrackObjectDistance::ComputeDistance3D(
    const PbfSensorObjectPtr &fused_object,
    const PbfSensorObjectPtr &sensor_object, const Eigen::Vector3d &ref_pos,
    int range) {
  const ObjectPtr &obj = fused_object->object;
  if (obj == nullptr) {
    AERROR << "Object is nullptr.";
    return (std::numeric_limits<float>::max)();
  }
  const PolygonDType &fused_poly = obj->polygon;
  Eigen::Vector3d fused_poly_center(0, 0, 0);
  bool fused_state =
      ComputePolygonCenter(fused_poly, ref_pos, range, &fused_poly_center);
  if (!fused_state) {
    AERROR << "fail to compute polygon center! fused polygon size:"
           << fused_poly.size();
    return (std::numeric_limits<float>::max)();
  }

  const ObjectPtr obj2 = sensor_object->object;
  if (obj2 == nullptr) {
    AERROR << "Object is nullptr.";
    return (std::numeric_limits<float>::max)();
  }
  const PolygonDType &sensor_poly = obj2->polygon;
  Eigen::Vector3d sensor_poly_center(0, 0, 0);
  bool sensor_state =
      ComputePolygonCenter(sensor_poly, ref_pos, range, &sensor_poly_center);

  if (!sensor_state) {
    AERROR << "fail to compute sensor polygon center: polygon size:"
           << sensor_poly.size();
    return (std::numeric_limits<float>::max)();
  }

  double fusion_timestamp = fused_object->timestamp;
  double sensor_timestamp = sensor_object->timestamp;
  double time_diff = sensor_timestamp - fusion_timestamp;
  fused_poly_center(0) += obj->velocity(0) * time_diff;
  fused_poly_center(1) += obj->velocity(1) * time_diff;
  float distance =
      ComputeEuclideanDistance(fused_poly_center, sensor_poly_center);
  return distance;
}

float PbfTrackObjectDistance::ComputeEuclideanDistance(
    const Eigen::Vector3d &des, const Eigen::Vector3d &src) {
  Eigen::Vector3d diff_pos = des - src;
  float distance =
      std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum());
  return distance;
}

bool PbfTrackObjectDistance::ComputePolygonCenter(const PolygonDType &polygon,
                                                  Eigen::Vector3d *center) {
  int size = polygon.size();
  if (size == 0) {
    return false;
  }
  *center = Eigen::Vector3d(0, 0, 0);
  for (int i = 0; i < size; ++i) {
    const auto &point = polygon.points[i];
    (*center)[0] += point.x;
    (*center)[1] += point.y;
  }
  *center /= size;
  return true;
}

bool PbfTrackObjectDistance::ComputePolygonCenter(
    const PolygonDType &polygon, const Eigen::Vector3d &ref_pos, int range,
    Eigen::Vector3d *center) {
  PolygonDType polygon_part;
  std::map<double, int> distance2idx;
  for (size_t idx = 0; idx < polygon.size(); ++idx) {
    const auto &point = polygon.points[idx];
    double distance =
        sqrt(pow(point.x - ref_pos(0), 2) + pow(point.y - ref_pos(1), 2));
    distance2idx.insert(std::make_pair(distance, idx));
  }

  int size = distance2idx.size();
  int nu = std::max(range, size / range + 1);
  nu = std::min(nu, size);
  int count = 0;
  for (auto it = distance2idx.begin(); it != distance2idx.end() && count < nu;
       ++it, ++count) {
    polygon_part.push_back(polygon[it->second]);
  }
  return ComputePolygonCenter(polygon_part, center);
}

}  // namespace perception
}  // namespace apollo
