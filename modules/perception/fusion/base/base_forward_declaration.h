/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <memory>

namespace apollo {
namespace perception {
namespace fusion {

struct SensorFrameHeader;
typedef std::shared_ptr<SensorFrameHeader> SensorFrameHeaderPtr;
typedef std::shared_ptr<const SensorFrameHeader> SensorFrameHeaderConstPtr;

class SensorFrame;
typedef std::shared_ptr<SensorFrame> SensorFramePtr;
typedef std::shared_ptr<const SensorFrame> SensorFrameConstPtr;

class Sensor;
typedef std::shared_ptr<Sensor> SensorPtr;
typedef std::shared_ptr<const Sensor> SensorConstPtr;

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
