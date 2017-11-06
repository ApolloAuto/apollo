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
 
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_monitor.h"

namespace apollo {
namespace perception {

PbfSensorMonitor::PbfSensorMonitor() {
}

bool PbfSensorMonitor::init() {
    return true;
}

void PbfSensorMonitor::update(const std::string& sensor_id, 
    double capture_time, double detection_time) {
    
    //TODO
    std::map<std::string, SensorStatus>::iterator it = _sensor_states.find(sensor_id);
    if (it == _sensor_states.end()) {
        SensorStatus status;
        status.sensor_id = sensor_id;
        status.latest_capture_time = capture_time;
        status.latest_detection_time = detection_time;
        status.latest_latency = detection_time - capture_time;
        _sensor_states[sensor_id] = status;
    } else {
        it->second.latest_capture_time = capture_time;
        it->second.latest_detection_time = detection_time;
        it->second.latest_latency = detection_time - capture_time;
    }
}

} // namespace perception
} // namespace apollo