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
#include "gtest/gtest.h"

#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/multi_sensor_fusion/base/sensor.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_object.h"
#include "modules/perception/multi_sensor_fusion/base/track.h"
#include "modules/perception/multi_sensor_fusion/fusion/dummy/dummy_algorithms.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(DummyFusionSystemTest, test) {
  FusionInitOptions init_options;
  DummyFusionSystem system;
  EXPECT_TRUE(system.Init(init_options));

  base::FramePtr frame(new base::Frame());
  frame->sensor_info.name = "velodyne64";
  frame->objects.resize(2);
  frame->objects[0].reset(new base::Object());
  frame->objects[1].reset(new base::Object());
  EXPECT_FALSE(system.Fuse(frame, nullptr));
  std::vector<base::ObjectPtr> objects;
  EXPECT_TRUE(system.Fuse(frame, &objects));
}

TEST(DummyDataAssociationTest, test) {}

TEST(DummyTrackerTest, test) {}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
