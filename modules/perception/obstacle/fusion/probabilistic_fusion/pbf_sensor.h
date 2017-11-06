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
 
#ifndef ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_H
#define ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_H
#include <deque>
#include "modules/common/macro.h"
#include "modules/common/log.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"

namespace apollo {
namespace perception {

class PbfSensor {
public:
    explicit PbfSensor(const SensorType& type, const std::string& sensor_id);
    ~PbfSensor();

    /**@brief query frames whose time stamp is in range (_latest_fused_time_stamp, time_stamp]*/
    void query_latest_frames(double time_stamp, std::vector<PbfSensorFramePtr>* frames);

    /**@brief query latest frame whose time stamp is in range (_latest_fused_time_stamp, time_stamp]*/
    PbfSensorFramePtr query_latest_frame(double time_stamp);

    /**@brief add a frame objects*/
    void add_frame(const SensorObjects& frame);

    /**@brief query pose at time_stamp, return false if not found*/
    bool get_pose(double time_stamp, Eigen::Matrix4d* pose);

    static void set_max_cached_frame_number(int number) {
        _s_max_cached_frame_number = number;
    }
private:
    DISALLOW_COPY_AND_ASSIGN(PbfSensor);
    PbfSensor();

protected:
    /**@brief cached frames in FIFO*/
    std::deque<PbfSensorFramePtr>  _frames;

    std::string                    _sensor_id;
    SensorType                     _sensor_type;

    /**@brief max size of _frames*/
    static int                     _s_max_cached_frame_number;

    double                         _latest_query_timestamp;
};

} // namespace perception
} // namespace apollo

#endif
