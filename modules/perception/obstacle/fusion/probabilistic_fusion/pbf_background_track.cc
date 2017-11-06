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
double PbfBackgroundTrack::s_max_invisible_period_ = 0.5;
void PbfBackgroundTrack::SetMaxInvisiblePeriod(double period) {
    s_max_invisible_period_ = period;
}
PbfBackgroundTrack::PbfBackgroundTrack(PbfSensorObjectPtr obj) {
    idx_ = PbfTrack::GetNextTrackId();
    fused_timestamp_ = obj->timestamp;
    fused_object_.reset(new PbfSensorObject());
    fused_object_->clone(*obj);
    age_ = 0;
    invisible_period_ = 0;
    tracking_period_ = 0.0;
    is_dead_ = false;
}
PbfBackgroundTrack::~PbfBackgroundTrack() {
}
void PbfBackgroundTrack::UpdateWithSensorObject(PbfSensorObjectPtr obj) {
    fused_object_->clone(*obj);
    invisible_period_ = 0;
    tracking_period_ += obj->timestamp - fused_timestamp_;
    fused_timestamp_ = obj->timestamp;
    fused_object_->timestamp = obj->timestamp;
}
void PbfBackgroundTrack::UpdateWithoutSensorObject(const SensorType &sensor_type,
                                                   const std::string &sensor_id, double timestamp) {
    
    invisible_period_ = timestamp - fused_timestamp_;
    if (invisible_period_ > s_max_invisible_period_) {
        is_dead_ = true;
    }
}
PbfSensorObjectPtr PbfBackgroundTrack::GetFusedObject() {
    return fused_object_;
}
bool PbfBackgroundTrack::AbleToPublish() const {
    double invisible_period_threshold = 0.01;
    if (invisible_period_ > invisible_period_threshold) {
        return false;
    }
    return true;
}

} // namespace perception
} // namespace apollo