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
 
#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_MONITOR_H_
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_MONITOR_H_
#include <string>
#include <map>
#include "modules/common/macro.h"

namespace apollo {
namespace perception {

struct SensorStatus {
    std::string sensor_id;
    double latest_capture_time = 0.0;
    double latest_detection_time = 0.0;
    int latest_obstacle_number = 0;
    double latest_latency = 0.0;
}; 

class PbfSensorMonitor{
public:
    PbfSensorMonitor();

    bool Init();

    void Update(const std::string &sensor_id, double capture_time, double detection_time);

protected:
    std::map<std::string, SensorStatus> sensor_states_;

private:
    DISALLOW_COPY_AND_ASSIGN(PbfSensorMonitor);
};

} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_MONITOR_H_
