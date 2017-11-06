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

#include <limits>
#include "boost/format.hpp"
#include "Eigen/StdVector"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"
#include "modules/common/log.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d);

namespace apollo {
namespace perception {

PbfTrackObjectDistance::PbfTrackObjectDistance() {}

PbfTrackObjectDistance::~PbfTrackObjectDistance() {}

float PbfTrackObjectDistance::Compute(const PbfTrackPtr &fused_track,
                                      const PbfSensorObjectPtr &sensor_object,
                                      const TrackObjectDistanceOptions &options) {
    const SensorType& sensor_type = sensor_object->sensor_type;
    ADEBUG << "sensor type: " << sensor_type;
    PbfSensorObjectPtr fused_object = fused_track->GetFusedObject();
    if (fused_object == nullptr) {
        ADEBUG << "fused object is nullptr";
        return (std::numeric_limits<float>::max)();
    }

    Eigen::Vector3d* ref_point = options.ref_point;
    if (ref_point == nullptr) {
        AERROR << "reference point is nullptr";
        return (std::numeric_limits<float>::max)();
    }

    float distance = (std::numeric_limits<float>::max)();
    const PbfSensorObjectPtr& lidar_object = fused_track->GetLatestLidarObject();
    const PbfSensorObjectPtr& radar_object = fused_track->GetLatestRadarObject();
    if (is_lidar(sensor_type)) {
        if (lidar_object != nullptr) {
            distance = ComputeVelodyne64Velodyne64(fused_object, sensor_object, *ref_point);
        } else if (radar_object != nullptr) {
            distance = ComputeVelodyne64Radar(sensor_object, fused_object, *ref_point);
        } else {
            AWARN << "All of the objects are nullptr";
        }
    } else if (is_radar(sensor_type)) {
        if (lidar_object != nullptr) {
            distance = ComputeVelodyne64Radar(fused_object, sensor_object, *ref_point);
        } else if (radar_object != nullptr) {
            distance = std::numeric_limits<float>::max();
        //    distance = compute_radar_radar(fused_object, sensor_object, *ref_point);
        } else {
            AWARN << "All of the objects are nullptr";
        }
    } else {
        AERROR << "fused sensor type is not support";
    }
    return distance;
}

float PbfTrackObjectDistance::ComputeVelodyne64Velodyne64(const PbfSensorObjectPtr &fused_object,
                                                          const PbfSensorObjectPtr &sensor_object,
                                                          const Eigen::Vector3d &ref_pos,
                                                          int range) {
    float distance = ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
    ADEBUG << "compute_velodyne64_velodyne64 distance: " << distance;
    return distance;
}

float PbfTrackObjectDistance::ComputeVelodyne64Radar(const PbfSensorObjectPtr &fused_object,
                                                     const PbfSensorObjectPtr &sensor_object,
                                                     const Eigen::Vector3d &ref_pos,
                                                     int range) {
    float distance = ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
    ADEBUG << "compute_velodyne64_radar distance " << distance;
    return distance;
}

float PbfTrackObjectDistance::ComputeRadarRadar(const PbfSensorObjectPtr &fused_object,
                                                const PbfSensorObjectPtr &sensor_object,
                                                const Eigen::Vector3d &ref_pos,
                                                int range) {
    float distance = ComputeDistance3D(fused_object, sensor_object, ref_pos, range);
    ADEBUG << "compute_radar_radar distance " << distance;
    return distance;
}

float PbfTrackObjectDistance::ComputeDistance3D(const PbfSensorObjectPtr &fused_object,
                                                const PbfSensorObjectPtr &sensor_object,
                                                const Eigen::Vector3d &ref_pos,
                                                int range) {
    const ObjectPtr& obj = fused_object->object;
    if (obj == nullptr) {
        AERROR << "Object is nullptr.";
        return (std::numeric_limits<float>::max)();
    }
    const PolygonDType& fused_poly = obj->polygon;
    Eigen::Vector3d fused_poly_center(0, 0, 0);
    bool fused_state = ComputePolygonCenter(fused_poly, ref_pos, range, fused_poly_center);
    if (!fused_state) {
        AERROR << "fail to compute polygon center! fused polygon size:" << fused_poly.size();
        return (std::numeric_limits<float>::max)();
    }

    const ObjectPtr obj2 = sensor_object->object;
    if (obj2 == nullptr) {
        AERROR << "Object is nullptr.";
        return (std::numeric_limits<float>::max)();
    }
    const PolygonDType& sensor_poly = obj2->polygon;
    Eigen::Vector3d sensor_poly_center(0, 0, 0);
    bool sensor_state = ComputePolygonCenter(sensor_poly, ref_pos, range, sensor_poly_center);

    if (!sensor_state) {
        AERROR << "fail to compute sensor polygon center: polygon size:" << sensor_poly.size();
        return (std::numeric_limits<float>::max)();
    }

    double fusion_timestamp = fused_object->timestamp;
    double sensor_timestamp = sensor_object->timestamp;
    double time_diff = sensor_timestamp - fusion_timestamp;
    fused_poly_center(0) += obj->velocity(0) * time_diff;
    fused_poly_center(1) += obj->velocity(1) * time_diff;
    float distance = ComputeEuclideanDistance(fused_poly_center, sensor_poly_center);
    return distance;
}

float PbfTrackObjectDistance::ComputeEuclideanDistance(const Eigen::Vector3d &des,
                                                       const Eigen::Vector3d &src) {
    Eigen::Vector3d diff_pos = des - src;
    float distance = std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum());
    return distance;
}

bool PbfTrackObjectDistance::ComputePolygonCenter(const PolygonDType &polygon,
                                                  Eigen::Vector3d &center) {
    int size = polygon.size();
    if (size == 0) {
        return false;
    }
    center = Eigen::Vector3d(0, 0, 0);
    for (int i = 0; i < size; ++i) {
       const auto& point = polygon.points[i];
       center[0] += point.x;
       center[1] += point.y;
    }
    center /= size;
    return true;
}

bool PbfTrackObjectDistance::ComputePolygonCenter(const PolygonDType &polygon,
                                                  const Eigen::Vector3d &ref_pos,
                                                  int range,
                                                  Eigen::Vector3d &center) {
    PolygonDType polygon_part;
    std::map<double, int> distance2idx;
    for (size_t idx = 0; idx < polygon.size(); ++idx) {
        const auto& point = polygon.points[idx];
        double distance = sqrt(pow(point.x - ref_pos(0), 2) +
                           pow(point.y - ref_pos(1), 2));
        distance2idx.insert(std::make_pair(distance, idx));
    }

    int size = distance2idx.size();
    // TODO
    int nu = std::max(range, size / range + 1);
    nu = std::min(nu, size);
    int count = 0;
    std::map<double, int>::iterator it = distance2idx.begin();
    for (; it!= distance2idx.end(), count < nu; ++it, ++count) {
        polygon_part.push_back(polygon[it->second]);
    }
    bool state = ComputePolygonCenter(polygon_part, center);
    return state;
}

} // namespace perception
} // namespace apollo
