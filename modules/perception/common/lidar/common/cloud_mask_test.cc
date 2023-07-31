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
#include "modules/perception/common/lidar/common/cloud_mask.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::AttributePointCloud;
using base::PointF;

TEST(LidarCloudMaskTest, lidar_cloud_mask_test) {
  CloudMask mask;
  mask.Set(10, 0);
  EXPECT_EQ(mask.size(), 10);
  EXPECT_EQ(mask.mask().size(), 10);
  const CloudMask& const_mask = mask;
  for (size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(mask[static_cast<int>(i)], 0);
    EXPECT_EQ(const_mask[static_cast<int>(i)], 0);
  }
  mask.Fill(1);
  for (size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(mask[static_cast<int>(i)], 1);
    EXPECT_EQ(const_mask[static_cast<int>(i)], 1);
  }
  EXPECT_EQ(mask.ValidIndicesCount(), 10);
  mask[0] = 0;
  mask.Flip();
  EXPECT_EQ(mask.ValidIndicesCount(), 1);
  mask.clear();
  EXPECT_EQ(mask.size(), 0);

  AttributePointCloud<PointF> source_cloud;
  source_cloud.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    source_cloud[i].x = static_cast<float>(i);
  }

  std::vector<int> indices = {1, 2, 3};
  std::vector<int> indices_of_indices = {1};

  base::PointIndices p_indices;
  base::PointIndices p_indices_of_indices;
  p_indices.indices = indices;
  p_indices_of_indices.indices = indices_of_indices;

  mask.Set(source_cloud.size(), 0);
  mask.AddIndices(p_indices);
  EXPECT_EQ(mask.ValidIndicesCount(), 3);
  mask.RemoveIndices(p_indices);
  mask.AddIndicesOfIndices(p_indices, p_indices_of_indices);
  EXPECT_EQ(mask.ValidIndicesCount(), 1);

  AttributePointCloud<PointF> target_cloud;
  mask.GetValidCloud(source_cloud, nullptr);
  mask.GetValidCloud(source_cloud, &target_cloud);
  EXPECT_EQ(target_cloud.size(), 1);
  EXPECT_EQ(target_cloud[0].x, 2.f);

  CloudMask another_mask;
  mask.GetValidMask(nullptr);
  mask.GetValidMask(&another_mask);
  EXPECT_EQ(another_mask.size(), 1);
  another_mask.ResetValue(1, 2);
  EXPECT_EQ(another_mask[0], 2);
  mask.RemoveIndicesOfIndices(p_indices, p_indices_of_indices);
  EXPECT_EQ(mask.ValidIndicesCount(), 0);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
