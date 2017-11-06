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
 
#ifndef ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_MANAGER_H
#define ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_MANAGER_H
#include <queue>
#include "modules/common/macro.h"
#include "modules/common/log.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor.h"

namespace apollo {
namespace perception {

class PbfSensorManager {
public:
    static PbfSensorManager* instance();
    ~PbfSensorManager();

    void add_sensor_measurements(const SensorObjects& objects);

    void get_latest_sensor_frames(double time_stamp, const std::string& sensor_id, 
        std::vector<PbfSensorFramePtr>* frames);
    
    /**@brief query one closest sensor frame for each sensor between last query timestamp and 
       current timestamp, stored in ascending order of the frame timestamp */
    void get_latest_frames(const double time_stamp, 
        std::vector<PbfSensorFramePtr>* frames);

    PbfSensor* get_sensor(const std::string& sensor_id);

    bool get_pose(const std::string& sensor_id, double time_stamp, Eigen::Matrix4d* pose);

protected:
    bool init();

private:
    PbfSensorManager();

    /**@brief sensor_id based key*/
    std::map<std::string, PbfSensor*> _sensors;
private:
    DISALLOW_COPY_AND_ASSIGN(PbfSensorManager);
};

} // namespace perception
} // namespace apollo

#endif