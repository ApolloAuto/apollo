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
#include "modules/perception/base/object.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace base {

TEST(ObjectTest, object_construct_test) {
  Object object;

  EXPECT_EQ(object.id, -1);
  EXPECT_LT((object.direction - Eigen::Vector3f(1, 0, 0)).norm(), 1.0e-6);
  EXPECT_LT(fabs(object.theta), 1.0e-6);
  EXPECT_LT(fabs(object.theta_variance), 1.0e-6);
  EXPECT_EQ(object.type, ObjectType::UNKNOWN);
  EXPECT_EQ(object.type_probs.size(),
            static_cast<int>(ObjectType::MAX_OBJECT_TYPE));
  EXPECT_EQ(object.track_id, -1);
  EXPECT_LT(fabs(object.tracking_time), 1.0e-6);
  EXPECT_LT(fabs(object.latest_tracked_time), 1.0e-6);
  EXPECT_LT(fabs(object.confidence - 1.0f), 1.0e-6);

  std::string str_object = object.ToString();

  Object cp_object(object);
  std::string str_cp_object = cp_object.ToString();
  EXPECT_EQ(str_object, str_cp_object);

  Object assignment_object;
  assignment_object = object;
  std::string str_assignment_object = assignment_object.ToString();
  EXPECT_EQ(str_assignment_object, str_object);
}

TEST(ObjectTest, object_reset_test) {
  Object object;
  std::string str_obj = object.ToString();

  object.id = 1;
  object.polygon.resize(2);
  object.direction = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  object.theta = static_cast<float>(M_PI / 2);
  object.theta_variance = 0.0f;
  object.center = Eigen::Vector3d(100, 0.0, 0.0);
  object.center_uncertainty << 0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f,
      0.01f;

  object.size = Eigen::Vector3f(3.0f, 4.0f, 5.0f);
  object.size_variance = Eigen::Vector3f(0.01f, 0.0f, 0.0f);
  object.anchor_point = Eigen::Vector3d(100.0, 0.0, 0.0);
  object.type = ObjectType::BICYCLE;
  object.type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0.3f);

  object.confidence = 0.6f;

  object.track_id = 1;
  object.velocity = Eigen::Vector3f(3.0f, 0.0f, 0.0f);
  object.velocity_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f;
  object.tracking_time = 0.6;
  object.latest_tracked_time = 12345678986.345;
  object.acceleration = Eigen::Vector3f(3.0f, 0.0f, 0.0f);
  object.acceleration_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f;

  object.Reset();
  std::string reset_obj_str = object.ToString();
  EXPECT_EQ(str_obj, reset_obj_str);
}

}  //  namespace base
}  //  namespace perception
}  //  namespace apollo
