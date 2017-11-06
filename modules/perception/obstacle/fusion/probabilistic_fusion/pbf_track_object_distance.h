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

#ifndef ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_OBJECT_DISTANCE_H
#define ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_OBJECT_DISTANCE_H

#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

struct TrackObjectDistanceOptions {
    Eigen::Vector3d* ref_point = nullptr;
};
class PbfTrackObjectDistance{
public:
    PbfTrackObjectDistance();
    virtual ~PbfTrackObjectDistance();

    float compute(const PbfTrackPtr& fused_track,
        const PbfSensorObjectPtr& sensor_object,
        const TrackObjectDistanceOptions& options);

protected:
    float compute_velodyne64_velodyne64(const PbfSensorObjectPtr& fused_object,
                              const PbfSensorObjectPtr& sensor_object,
                              const Eigen::Vector3d& ref_pos,
                              int range = 3);
    float compute_velodyne64_radar(const PbfSensorObjectPtr& fused_object,
                              const PbfSensorObjectPtr& sensor_object,
                              const Eigen::Vector3d& ref_pos,
                              int range = 3);
    float compute_radar_radar(const PbfSensorObjectPtr& fused_object,
                              const PbfSensorObjectPtr& sensor_object,
                              const Eigen::Vector3d& ref_pos,
                              int range = 3);

    float compute_distance_3d(const PbfSensorObjectPtr& fused_object,
                              const PbfSensorObjectPtr& sensor_object,
                              const Eigen::Vector3d& ref_pos,
                              int range);
    float compute_euclidean_distance(const Eigen::Vector3d& des,
                                     const Eigen::Vector3d& src);
    bool compute_polygon_center(const PolygonDType& polygon,
                                Eigen::Vector3d& center);
    bool compute_polygon_center(const PolygonDType& polygon,
                                const Eigen::Vector3d& ref_pos,
                                int range,
                                Eigen::Vector3d& center);
private:
    DISALLOW_COPY_AND_ASSIGN(PbfTrackObjectDistance);
};
} // namespace perception
} // namespace apollo

#endif
