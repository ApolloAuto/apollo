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
 
#ifndef ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H
#define ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H
#include <utility>
#include <vector>
#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

namespace apollo {
namespace perception {

class PbfBaseMotionFusion {
public:
    PbfBaseMotionFusion() : _name("PbfBaseMotionFusion"), _initialized(false) {
    }
    virtual ~PbfBaseMotionFusion() {}

    // @brief initialize the state of filter
    // @params[IN] anchor_point: initial anchor point for filtering
    // @params[IN] velocity: initial velocity for filtering
    // @return nothing
    virtual void initialize(const Eigen::Vector3d& anchor_point,
        const Eigen::Vector3d& velocity) = 0;

    // @brief initialize state of the filter
    // @params[IN] new_object: initial object for filtering
    // @return nothing
    virtual void initialize(const PbfSensorObjectPtr new_object) = 0;

    // @brief predict the state of filter
    // @params[OUT] anchor_point:  predicted anchor point for filtering
    // @params[OUT] velocity: predicted velocity
    // @params[IN] time_diff: time interval from last update
    // @return nothing
    virtual void predict(Eigen::Vector3d& anchor_point,
        Eigen::Vector3d& velocity, const double time_diff) = 0;

    // @brief update with measurements
    // @params[IN] new_object: new object for current update
    // @params[IN] time_diff: time interval from last update;
    // @return nothing
    virtual void update_with_object(const PbfSensorObjectPtr new_object,
        const double time_diff) = 0;

    // @brief update without measurements
    // @params[IN] time_diff: time interval from last update
    // @return nothing
    virtual void update_without_object(const double time_diff) = 0;

    // @brief get current state of the filter
    // @params[OUT] anchor_point: current anchor_point
    // @params[OUT] velocity: current velocity
    // @return nothing
    virtual void get_state(Eigen::Vector3d& anchor_point, Eigen::Vector3d& velocity) = 0;

    std::string name() {
        return _name;
    }

    // @brief check if filter has been initialized
    // @return initialization status 
    bool initialized() const {
        return _initialized;
    }

protected:
    std::string _name;
    bool _initialized;    
};

} // namespace perception
} // namespace apollo

#endif
