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

#include "modules/perception/obstacle/base/object.h"

#include <vector>

#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

using std::vector;

TEST(ObjectTest, test_Object) {
  Object obj;
  obj.id = 1;
  obj.track_id = 2;
  obj.direction << 1.0, 2.0, 3.0;
  obj.center << 4.0, 5.0, 6.0;
  obj.velocity << 7.0, 8.0, 9.0;
  obj.theta = 0.0;
  obj.length = 0.1;
  obj.width = 0.2;
  obj.height = 0.3;
  obj.type = ObjectType::BICYCLE;
  obj.tracking_time = 10.0;
  obj.latest_tracked_time = 123456.7;
  Object obj2(obj);
  EXPECT_EQ(obj.id, obj2.id);
  EXPECT_EQ(obj.track_id, obj2.track_id);
  EXPECT_FLOAT_EQ(obj.theta, obj2.theta);
  EXPECT_FLOAT_EQ(obj.length, obj2.length);
  EXPECT_FLOAT_EQ(obj.width, obj2.width);
  EXPECT_FLOAT_EQ(obj.height, obj2.height);
  EXPECT_FLOAT_EQ(obj.tracking_time, obj2.tracking_time);
  EXPECT_FLOAT_EQ(obj.latest_tracked_time, obj2.latest_tracked_time);
}

TEST(ObjectTest, test_operator_eq) {
  Object obj;
  obj.id = 1;
  obj.track_id = 2;
  obj.direction << 1.0, 2.0, 3.0;
  obj.center << 4.0, 5.0, 6.0;
  obj.velocity << 7.0, 8.0, 9.0;
  obj.theta = 0.0;
  obj.length = 0.1;
  obj.width = 0.2;
  obj.height = 0.3;
  obj.type = ObjectType::BICYCLE;
  obj.tracking_time = 10.0;
  obj.latest_tracked_time = 123456.7;
  Object obj2;
  obj2 = obj;
  EXPECT_EQ(obj.id, obj2.id);
  EXPECT_EQ(obj.track_id, obj2.track_id);
  EXPECT_FLOAT_EQ(obj.theta, obj2.theta);
  EXPECT_FLOAT_EQ(obj.length, obj2.length);
  EXPECT_FLOAT_EQ(obj.width, obj2.width);
  EXPECT_FLOAT_EQ(obj.height, obj2.height);
  EXPECT_FLOAT_EQ(obj.tracking_time, obj2.tracking_time);
  EXPECT_FLOAT_EQ(obj.latest_tracked_time, obj2.latest_tracked_time);
}

TEST(ObjectTest, test_clone) {
  Object obj;
  obj.id = 1;
  obj.track_id = 2;
  obj.direction << 1.0, 2.0, 3.0;
  obj.center << 4.0, 5.0, 6.0;
  obj.velocity << 7.0, 8.0, 9.0;
  obj.theta = 0.0;
  obj.length = 0.1;
  obj.width = 0.2;
  obj.height = 0.3;
  obj.type = ObjectType::BICYCLE;
  obj.tracking_time = 10.0;
  obj.latest_tracked_time = 123456.7;
  Object obj2;
  obj2.clone(obj);
  EXPECT_EQ(obj.id, obj2.id);
  EXPECT_EQ(obj.track_id, obj2.track_id);
  EXPECT_FLOAT_EQ(obj.theta, obj2.theta);
  EXPECT_FLOAT_EQ(obj.length, obj2.length);
  EXPECT_FLOAT_EQ(obj.width, obj2.width);
  EXPECT_FLOAT_EQ(obj.height, obj2.height);
  EXPECT_FLOAT_EQ(obj.tracking_time, obj2.tracking_time);
  EXPECT_FLOAT_EQ(obj.latest_tracked_time, obj2.latest_tracked_time);
}

TEST(ObjectTest, test_ToString) {
  Object object;
  AINFO << object.ToString();
}

TEST(ObjectTest, test_Serialize) {
  Object obj;
  obj.id = 1;
  obj.track_id = 2;
  obj.direction << 1.0, 2.0, 3.0;
  obj.center << 4.0, 5.0, 6.0;
  obj.velocity << 7.0, 8.0, 9.0;
  obj.theta = 0.0;
  obj.length = 0.1;
  obj.width = 0.2;
  obj.height = 0.3;
  obj.type = ObjectType::BICYCLE;
  obj.tracking_time = 10.0;
  obj.latest_tracked_time = 123456.7;
  obj.cloud = pcl_util::PointCloudPtr(new pcl_util::PointCloud);
  pcl_util::Point temp;
  temp.x = 1;
  temp.y = 2;
  temp.z = 3;
  obj.cloud->push_back(temp);

  PolygonDType polygon;
  for (int i = 1; i < 5; ++i) {
    pcl_util::PointD p;
    p.x = i;
    p.y = i;
    p.z = i;
    polygon.push_back(p);
  }
  obj.polygon = polygon;

  PerceptionObstacle pb_obj;
  obj.Serialize(&pb_obj);
  EXPECT_EQ(pb_obj.id(), 2);
  EXPECT_EQ(pb_obj.position().x(), 4.0);
  AINFO << "org obj:" << obj.ToString();
  AINFO << "pb obj:" << pb_obj.ShortDebugString();
}

TEST(ObjectTest, test_Deserialize) {
  Object obj;
  PerceptionObstacle pb_obj;
  pb_obj.set_id(1);
  pb_obj.set_theta(0.1);
  Point* pos = pb_obj.mutable_position();
  pos->set_x(10.1);
  pos->set_y(20.1);
  pos->set_z(1.1);
  pb_obj.set_length(2.0);
  pb_obj.set_width(1.0);
  pb_obj.set_height(1.5);
  pb_obj.set_tracking_time(20.1);
  pb_obj.set_type(PerceptionObstacle::BICYCLE);
  pb_obj.set_timestamp(1147012345.678);

  PolygonDType polygon;
  for (int i = 1; i < 5; ++i) {
    pcl_util::PointD p;
    p.x = i;
    p.y = i;
    p.z = i;
    polygon.push_back(p);
  }

  obj.Deserialize(pb_obj);
  EXPECT_EQ(obj.track_id, 1);
  EXPECT_EQ(obj.theta, 0.1);
  EXPECT_EQ(obj.center(0), 10.1);
  EXPECT_EQ(obj.center(1), 20.1);
  EXPECT_EQ(obj.center(2), 1.1);
  EXPECT_EQ(obj.length, 2.0);
  EXPECT_EQ(obj.width, 1.0);
  EXPECT_EQ(obj.height, 1.5);
  EXPECT_EQ(obj.type, ObjectType::BICYCLE);
  EXPECT_EQ(obj.tracking_time, 20.1);
  EXPECT_EQ(obj.latest_tracked_time, 1147012345.678);
}

}  // namespace perception
}  // namespace apollo
