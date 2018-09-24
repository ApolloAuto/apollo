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
#include <gtest/gtest.h>
#include <limits>
#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud_types.h"
#include "modules/perception/common/geometry/roi_filter.h"
namespace apollo {
namespace perception {
namespace common {

TEST(IsPtInRoiTest, test_roi) {
  base::HdmapStructPtr hdmap = base::HdmapStructPtr(new base::HdmapStruct());
  hdmap->junction_polygons.resize(1);
  base::PointD pt;
  pt.x = -1.0;
  pt.y = -1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  pt.x = 1.0;
  pt.y = -1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  pt.x = 0.0;
  pt.y = 1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  bool flag = false;
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = 0.0;
  const base::HdmapStructConstPtr hdmo_const = hdmap;
  flag = IsPtInRoi(hdmo_const, pt);
  EXPECT_TRUE(flag);
  pt.x = 10.0;
  pt.y = 10.0;
  pt.z = 10.0;
  flag = IsPtInRoi(hdmo_const, pt);
  EXPECT_FALSE(flag);
}

TEST(IsObjectInRoiTest, test_roi) {
  base::HdmapStructPtr hdmap = base::HdmapStructPtr(new base::HdmapStruct());
  hdmap->junction_polygons.resize(1);
  base::PointD pt;
  pt.x = -1.0;
  pt.y = -1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  pt.x = 1.0;
  pt.y = -1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  pt.x = 0.0;
  pt.y = 1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  bool flag = false;
  base::ObjectConstPtr obj = base::ObjectPtr(new base::Object());
  const base::HdmapStructConstPtr hdmo_const = hdmap;
  flag = IsObjectInRoi(hdmo_const, obj);
  EXPECT_TRUE(flag);
}

TEST(IsObjectBboxInRoiTest, test_roi) {
  base::HdmapStructPtr hdmap = base::HdmapStructPtr(new base::HdmapStruct());
  hdmap->junction_polygons.resize(1);
  base::PointD pt;
  pt.x = -1.0;
  pt.y = -1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  pt.x = 1.0;
  pt.y = -1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  pt.x = 0.0;
  pt.y = 1.0;
  pt.z = 0.0;
  hdmap->junction_polygons[0].push_back(pt);
  bool flag = false;
  base::ObjectConstPtr obj = base::ObjectPtr(new base::Object());
  const base::HdmapStructConstPtr hdmo_const = hdmap;
  flag = IsObjectBboxInRoi(hdmo_const, obj);
  EXPECT_TRUE(flag);

  base::ObjectPtr obj_unconst = base::ObjectPtr(new base::Object());
  obj_unconst->center[0] = 10.0;
  obj_unconst->center[1] = 10.0;
  obj_unconst->center[2] = 10.0;
  flag = IsObjectBboxInRoi(hdmo_const, obj_unconst);
  EXPECT_FALSE(flag);

  obj_unconst->direction[0] = 0.0;
  obj_unconst->center[0] = 0.0;
  obj_unconst->center[1] = 0.0;
  obj_unconst->center[2] = 0.0;
  flag = IsObjectBboxInRoi(hdmo_const, obj_unconst);
  EXPECT_TRUE(flag);
}

TEST(ObjectInRoiTest, test_roi) {
  base::HdmapStructPtr hdmap = nullptr;
  std::vector<base::ObjectPtr> objects;
  std::vector<base::ObjectPtr> valid_objects;

  base::ObjectPtr obj(new base::Object);
  obj->center = Eigen::Vector3d(0, 0, 0);
  objects.push_back(obj);
  ObjectInRoiCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 1);

  hdmap.reset(new base::HdmapStruct());
  valid_objects.clear();
  ObjectInRoiCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 1);

  hdmap->road_polygons.resize(1);
  base::PointD pt;
  pt.x = -1.0;
  pt.y = -1.0;
  pt.z = 0;
  hdmap->road_polygons[0].push_back(pt);
  pt.x = 1.0;
  pt.y = -1.0;
  pt.z = 0;
  hdmap->road_polygons[0].push_back(pt);
  pt.x = 0;
  pt.y = 1;
  pt.z = 0;
  hdmap->road_polygons[0].push_back(pt);
  valid_objects.clear();
  ObjectInRoiCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 1);

  hdmap->road_polygons[0][2].y = -0.1;
  valid_objects.clear();
  ObjectInRoiCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 0);
}

TEST(ObjectInRoiSlackTest, test_roi) {
  base::HdmapStructPtr hdmap = nullptr;
  std::vector<base::ObjectPtr> objects;
  std::vector<base::ObjectPtr> valid_objects;
  base::ObjectPtr obj(new base::Object);
  obj->center = Eigen::Vector3d(0, 0, 0);
  obj->direction = Eigen::Vector3f(1, 0, 0);
  objects.push_back(obj);

  ObjectInRoiSlackCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 1);

  hdmap.reset(new base::HdmapStruct());
  valid_objects.clear();
  ObjectInRoiSlackCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 1);

  hdmap->road_polygons.resize(1);
  base::PointD pt;
  pt.x = -1.0;
  pt.y = -1.0;
  pt.z = 0;
  hdmap->road_polygons[0].push_back(pt);
  pt.x = 1.0;
  pt.y = -1.0;
  pt.z = 0;
  hdmap->road_polygons[0].push_back(pt);
  pt.x = 0;
  pt.y = 1;
  pt.z = 0;
  hdmap->road_polygons[0].push_back(pt);
  valid_objects.clear();
  ObjectInRoiSlackCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 1);

  hdmap->road_polygons[0][2].y = -2;
  valid_objects.clear();
  ObjectInRoiSlackCheck(hdmap, objects, &valid_objects);
  EXPECT_EQ(valid_objects.size(), 0);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
