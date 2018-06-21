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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"

namespace apollo {
namespace perception {

PbfSensorObject::PbfSensorObject()
    : sensor_type(SensorType::UNKNOWN_SENSOR_TYPE),
      timestamp(0.0),
      invisible_period(0.0) {
  object.reset(new Object());
}

PbfSensorObject::PbfSensorObject(std::shared_ptr<Object> obj3d, SensorType type,
                                 double time)
    : sensor_type(type),
      timestamp(time),
      object(obj3d),
      invisible_period(0.0) {}

PbfSensorObject::~PbfSensorObject() {}

PbfSensorObject::PbfSensorObject(const PbfSensorObject &rhs) {
  sensor_type = rhs.sensor_type;
  sensor_id = rhs.sensor_id;
  timestamp = rhs.timestamp;
  object = rhs.object;
  invisible_period = rhs.invisible_period;
}

PbfSensorObject &PbfSensorObject::operator=(const PbfSensorObject &rhs) {
  sensor_type = rhs.sensor_type;
  sensor_id = rhs.sensor_id;
  timestamp = rhs.timestamp;
  object = rhs.object;
  invisible_period = rhs.invisible_period;
  return (*this);
}

void PbfSensorObject::clone(const PbfSensorObject &rhs) {
  sensor_type = rhs.sensor_type;
  sensor_id = rhs.sensor_id;
  timestamp = rhs.timestamp;
  invisible_period = rhs.invisible_period;
  if (object == nullptr) {
    object.reset(new Object());
  }
  object->clone(*(rhs.object));
}

}  // namespace perception
}  // namespace apollo
