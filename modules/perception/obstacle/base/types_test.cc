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

#include "modules/perception/obstacle/base/types.h"

#include "gtest/gtest.h"

#include "modules/common/log.h"

namespace apollo {
namespace perception {

TEST(TypesTest, test_GetSensorType) {
  EXPECT_EQ(GetSensorType(SensorType::VELODYNE_64), "velodyne_64");
  EXPECT_EQ(GetSensorType(SensorType::VELODYNE_16), "velodyne_16");
  EXPECT_EQ(GetSensorType(SensorType::RADAR), "radar");
  EXPECT_EQ(GetSensorType(SensorType::CAMERA), "camera");
  EXPECT_EQ(GetSensorType(SensorType::UNKNOWN_SENSOR_TYPE),
            "unknown_sensor_type");
  EXPECT_EQ(GetSensorType(SensorType(-1)), "");
}

}  // namespace perception
}  // namespace apollo
