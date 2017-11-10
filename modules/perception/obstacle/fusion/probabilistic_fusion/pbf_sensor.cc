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

int PbfSensor::s_max_cached_frame_number_ = 10;

PbfSensor::PbfSensor(const SensorType &type, const std::string &sensor_id) :
    sensor_id_(sensor_id), sensor_type_(type), latest_query_timestamp_(0.0) {
}

PbfSensor::~PbfSensor() {

}

void PbfSensor::QueryLatestFrames(double time_stamp, std::vector<PbfSensorFramePtr> *frames) {
  if (frames == nullptr) {
    return;
  }
  frames->clear();
  for (size_t i = 0; i < frames_.size(); i++) {
    if (frames_[i]->timestamp > latest_query_timestamp_ &&
        frames_[i]->timestamp <= time_stamp) {
      (*frames).push_back(frames_[i]);
    }
  }
  latest_query_timestamp_ = time_stamp;
}

PbfSensorFramePtr PbfSensor::QueryLatestFrame(double time_stamp) {

  PbfSensorFramePtr latest_frame = nullptr;
  for (size_t i = 0; i < frames_.size(); i++) {
    if (frames_[i]->timestamp > latest_query_timestamp_ &&
        frames_[i]->timestamp <= time_stamp) {
      latest_frame = frames_[i];
      latest_query_timestamp_ = frames_[i]->timestamp;
    }
  }
  return latest_frame;
}

void PbfSensor::AddFrame(const SensorObjects &frame) {
  //NOTE: keep empty frame for completeness
  PbfSensorFramePtr pbf_frame(new PbfSensorFrame());
  pbf_frame->timestamp = frame.timestamp;
  pbf_frame->sensor2world_pose = frame.sensor2world_pose;
  pbf_frame->sensor_type = frame.sensor_type;
  pbf_frame->sensor_id = GetSensorType(frame.sensor_type);
  pbf_frame->seq_num = frame.seq_num;

  pbf_frame->objects.resize(frame.objects.size());
  for (int i = 0; i < (int) frame.objects.size(); i++) {
    PbfSensorObjectPtr obj(new PbfSensorObject());
    obj->timestamp = frame.timestamp;
    obj->sensor_type = frame.sensor_type;
    obj->object->clone(*(frame.objects[i]));
    obj->sensor_id = GetSensorType(frame.sensor_type);
    pbf_frame->objects[i] = obj;
  }
  if ((int) (frames_.size()) > s_max_cached_frame_number_) {
    frames_.pop_front();
  }
  frames_.push_back(pbf_frame);
}

bool PbfSensor::GetPose(double time_stamp, Eigen::Matrix4d *pose) {
  if (pose == nullptr) {
    AERROR << "parameter pose is nullptr for output";
    return false;
  }

  for (int i = (int) frames_.size() - 1; i >= 0; i--) {
    double time_diff = time_stamp - frames_[i]->timestamp;
    if (fabs(time_diff) < 1.0e-3) {
      *pose = frames_[i]->sensor2world_pose;
      return true;
    }
  }
  AERROR << "Failed to find velodyne2world pose for timestamp: " << time_stamp;

  return false;
}

} // namespace perception
} // namespace apollo