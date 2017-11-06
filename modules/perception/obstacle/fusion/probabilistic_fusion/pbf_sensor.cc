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
 
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor.h"

namespace apollo {
namespace perception {

int PbfSensor::_s_max_cached_frame_number = 10;

PbfSensor::PbfSensor(const SensorType& type, const std::string& sensor_id): 
     _sensor_id(sensor_id), _sensor_type(type), _latest_query_timestamp(0.0) {
}

PbfSensor::~PbfSensor() {

}

void PbfSensor::query_latest_frames(double time_stamp, std::vector<PbfSensorFramePtr>* frames) {
    if (frames == nullptr) {
        return;
    }

    frames->clear();
    for (size_t i = 0; i < _frames.size(); i++) {
        if (_frames[i]->timestamp > _latest_query_timestamp && 
            _frames[i]->timestamp <= time_stamp) {
            (*frames).push_back(_frames[i]);
        }
    }
    _latest_query_timestamp = time_stamp;
}

PbfSensorFramePtr PbfSensor::query_latest_frame(double time_stamp) {

    PbfSensorFramePtr latest_frame = nullptr;
    for (size_t i = 0; i < _frames.size(); i++) {
        if (_frames[i]->timestamp > _latest_query_timestamp &&
            _frames[i]->timestamp <= time_stamp) {
            latest_frame = _frames[i];
            _latest_query_timestamp = _frames[i]->timestamp;
        }
    }
//    if (_sensor_type != CAMERA) {
//        _latest_query_timestamp = time_stamp;
//   }
//    _latest_query_timestamp = time_stamp;
    return latest_frame;
}

void PbfSensor::add_frame(const SensorObjects& frame) {
    //NOTE: keep empty frame for completeness
    PbfSensorFramePtr pbf_frame(new PbfSensorFrame());
    pbf_frame->timestamp = frame.timestamp;
    pbf_frame->sensor2world_pose = frame.sensor2world_pose;
    pbf_frame->sensor_type = frame.sensor_type;
    pbf_frame->sensor_id = GetSensorType(frame.sensor_type);
    pbf_frame->seq_num = frame.seq_num;

    pbf_frame->objects.resize(frame.objects.size());
    for (int i = 0; i < (int)frame.objects.size(); i++) {
        PbfSensorObjectPtr obj(new PbfSensorObject());
        obj->timestamp = frame.timestamp;
        obj->sensor_type = frame.sensor_type;
        obj->object->clone(*(frame.objects[i]));
        obj->sensor_id = GetSensorType(frame.sensor_type);
        pbf_frame->objects[i] = obj;
    }
    if (_frames.size() > _s_max_cached_frame_number) {
        _frames.pop_front();
    }
    _frames.push_back(pbf_frame);
}

bool PbfSensor::get_pose(double time_stamp, Eigen::Matrix4d* pose) {
    if (pose == nullptr) {
        AERROR << "parameter pose is nullptr for output";
        return false;
    }

    for (int i = (int)_frames.size() - 1; i >= 0; i--) {
        double time_diff = time_stamp - _frames[i]->timestamp;
        if (fabs(time_diff) < 1.0e-3) {
            *pose = _frames[i]->sensor2world_pose;
            return true;
        }
    }
    AERROR << "Failed to find velodyne2world pose for timestamp: " << time_stamp;

    return false;
}

} // namespace perception
} // namespace apollo