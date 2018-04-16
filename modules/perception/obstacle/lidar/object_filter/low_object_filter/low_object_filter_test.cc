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

#include "modules/perception/obstacle/lidar/object_filter/low_object_filter/low_object_filter.h"

#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/types.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {

void ConstructObject(std::vector<std::shared_ptr<Object> >* objects) {
  objects->resize(10);

  for (size_t i = 0; i < objects->size(); ++i) {
    Object obj;
    obj.id = i;
    pcl_util::PointCloudPtr cloud(new pcl_util::PointCloud);
    cloud->points.resize(10);
    for (size_t pi = 0; pi < 10; ++pi) {
      double num = pi * 1.0;
      cloud->points[pi].x = num * 1.5;
      cloud->points[pi].y = num + 3;
      if (i < 3 && i > 0) {
        cloud->points[pi].z = num / 150 - 3.5;
      } else {
        cloud->points[pi].z = num / 2 - 3.5;
      }
    }
    obj.cloud = cloud;
    objects->at(i) = std::make_shared<Object>(obj);
  }
}

TEST(LowObjectFilterTest, test_object_filter) {
  LowObjectFilter filter;

  ObjectFilterOptions object_filter_options;
  std::vector<std::shared_ptr<Object> >* objects
      = new std::vector<std::shared_ptr<Object> >();
  ConstructObject(objects);

  EXPECT_EQ(10, objects->size());

  filter.Init();
  filter.Filter(object_filter_options, objects);

  EXPECT_EQ(8, objects->size());
}

}  // namespace perception
}  // namespace apollo
