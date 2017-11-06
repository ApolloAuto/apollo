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
 
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_background_track.h"

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"
#include "boost/format.hpp"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/common/macro.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"
namespace apollo {
namespace perception {

/*class PbfBackgroundTrack*/
double PbfBackgroundTrack::_s_max_invisible_period = 0.5;
void PbfBackgroundTrack::set_max_invisible_period(double period) {
    _s_max_invisible_period = period;
}
PbfBackgroundTrack::PbfBackgroundTrack(PbfSensorObjectPtr obj) {
    _idx = PbfTrack::get_next_track_id();
    SensorType sensor_type = obj->sensor_type;
    std::string sensor_id = obj->sensor_id;
    _fused_timestamp = obj->timestamp;
    _fused_object.reset(new PbfSensorObject());
    _fused_object->clone(*obj);
    _age = 0;
    _invisible_period = 0;
    _tracking_period = 0.0;
    _is_dead = false;
}
PbfBackgroundTrack::~PbfBackgroundTrack() {
}
void PbfBackgroundTrack::update_with_sensor_object(PbfSensorObjectPtr obj) {
    _fused_object->clone(*obj);
    _invisible_period = 0;
    _tracking_period += obj->timestamp - _fused_timestamp;
    _fused_timestamp = obj->timestamp;
    _fused_object->timestamp = obj->timestamp;
}
void PbfBackgroundTrack::update_without_sensor_object(const SensorType& sensor_type,
    const std::string& sensor_id, double timestamp) {
    
    _invisible_period = timestamp - _fused_timestamp;
    if (_invisible_period > _s_max_invisible_period) {
        _is_dead = true;
    }
}
PbfSensorObjectPtr PbfBackgroundTrack::get_fused_object() {
    return _fused_object;
}
bool PbfBackgroundTrack::able_to_publish() const {
    double invisible_period_threshold = 0.01;
    if (_invisible_period > invisible_period_threshold) {
        return false;
    }
    return true;
}

} // namespace perception
} // namespace apollo