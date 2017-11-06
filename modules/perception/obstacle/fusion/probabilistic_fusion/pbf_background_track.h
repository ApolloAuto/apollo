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
 
#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BACKGROUND_TRACK_H_
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BACKGROUND_TRACK_H_

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"

namespace apollo {
namespace perception {

class PbfBackgroundTrack {
public:
    explicit PbfBackgroundTrack(PbfSensorObjectPtr obj);

    ~PbfBackgroundTrack();

    /**@brief Update track with sensor object */
    void UpdateWithSensorObject(PbfSensorObjectPtr obj);

    void UpdateWithoutSensorObject(const SensorType &sensor_type,
                                   const std::string &sensor_id, double timestamp);

    PbfSensorObjectPtr GetFusedObject();

    inline double GetFusedTimestamp() const {
        return fused_timestamp_;
    }

    inline int GetTrackId() const {
        return idx_;
    }

    inline double GetTrackingPeriod() const {
        return tracking_period_;
    }

    inline bool IsDead() const {
        return is_dead_;
    }

    bool AbleToPublish() const;

    static void SetMaxInvisiblePeriod(double period);

protected:
    PbfSensorObjectPtr    fused_object_;

    /**@brief time stamp of the track*/
    double                fused_timestamp_;

    int                   age_;
    double                tracking_period_;

    /**@brief global track id*/
    int                   idx_;
    double                invisible_period_;

    bool                  is_dead_;

    static double         s_max_invisible_period_;
private:
    PbfBackgroundTrack();
};

typedef std::shared_ptr<PbfBackgroundTrack> PbfBackgroundTrackPtr;

} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BACKGROUND_TRACK_H_
