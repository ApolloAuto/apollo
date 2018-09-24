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
#include <vector>
#include "modules/perception/base/point_cloud_types.h"
#include "modules/perception/common/point_cloud_processing/downsampling.h"
namespace apollo {
namespace perception {
namespace common {
TEST(PointCloudProcessingDownsamplingTest, downsampling_circular_test1) {
  base::PointF center_pt, tmp_pt;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  float radius = 2.f;
  float neighbour_dist = 0.f;
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PointCloud<base::PointF>);
  base::PolygonFTypePtr pc_out =
      base::PolygonFTypePtr(new base::PointCloud<base::PointF>);
  tmp_pt.x = 1.f;
  tmp_pt.y = 1.f;
  tmp_pt.z = 0.f;
  pc_in->push_back(tmp_pt);
  tmp_pt.x = 0.f;
  tmp_pt.y = 1.f;
  tmp_pt.z = 0.f;
  pc_in->push_back(tmp_pt);
  tmp_pt.x = 10.f;
  tmp_pt.y = 10.f;
  tmp_pt.z = 10.f;
  pc_in->push_back(tmp_pt);
  EXPECT_EQ(pc_in->size(), 3);
  EXPECT_EQ(pc_in->height(), 1);
  EXPECT_EQ(pc_in->width(), 3);
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingCircular(center_pt, radius, neighbour_dist, pc_in_const, pc_out);
  EXPECT_EQ(pc_out->size(), 2);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_circular_test2) {
  base::PointF center_pt, tmp_pt;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  float radius = 2.f;
  float neighbour_dist = 0.f;
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(4, 4, base::PointF()));
  base::PolygonFTypePtr pc_out =
      base::PolygonFTypePtr(new base::PointCloud<base::PointF>);
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 0.f;
      *(pc_in->at(i, j)) = tmp_pt;
    }
  }
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingCircular(center_pt, radius, neighbour_dist, pc_in_const, pc_out);
  EXPECT_EQ(pc_out->size(), 4);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_circular_test3) {
  base::PointF center_pt, tmp_pt;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  float radius = 3.f;
  float neighbour_dist = 1.1f;
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(4, 4, base::PointF()));
  base::PolygonFTypePtr pc_out =
      base::PolygonFTypePtr(new base::PointCloud<base::PointF>);
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 0.f;
      *(pc_in->at(i, j)) = tmp_pt;
    }
  }
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingCircular(center_pt, radius, neighbour_dist, pc_in_const, pc_out);
  EXPECT_EQ(pc_out->size(), 6);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_circular_org_all1) {
  base::PolygonFTypePtr pc_in = base::PolygonFTypePtr(new base::PolygonFType);
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  base::PointF center_pt, tmp_pt;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  for (float i = 1.f; i <= 128; i++) {
    tmp_pt.x = 1.f * i;
    tmp_pt.y = 1.f * i;
    tmp_pt.z = 1.f * i;
    pc_in->push_back(tmp_pt);
  }
  int smp_ratio = 1;
  float radius = 6.f;
  int velodyne_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingCircularOrgAll(center_pt, smp_ratio, radius, velodyne_model,
                             pc_in_const, pc_out);
  EXPECT_EQ(pc_out->size(), 4);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_circular_org_all2) {
  base::PolygonFTypePtr pc_in = base::PolygonFTypePtr(new base::PolygonFType);
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  base::PointF center_pt, tmp_pt;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  tmp_pt.x = 1.f;
  tmp_pt.y = 1.f;
  tmp_pt.z = 0.0f / 0.0f;
  pc_in->push_back(tmp_pt);
  int smp_ratio = 1;
  float radius = 6.f;
  int velodyne_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingCircularOrgAll(center_pt, smp_ratio, radius, velodyne_model,
                             pc_in_const, pc_out);
  EXPECT_EQ(pc_out->size(), 0);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_circular_org_partial1) {
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(64, 64, base::PointF()));
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  base::PointF center_pt, tmp_pt;
  typedef std::vector<std::pair<int, int>> VectorPair;
  VectorPair all_org_idx;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  for (size_t i = 0; i < 64; i++) {
    for (size_t j = 0; j < 64; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 1.f;
      *(pc_in->at(i, j)) = tmp_pt;
      all_org_idx.push_back(std::make_pair(i, j));
    }
  }
  int org_num = 16;
  int smp_ratio = 8;
  float radius = 100.f;
  int velodyne_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingCircularOrgPartial(center_pt, org_num, smp_ratio, radius,
                                 velodyne_model, pc_in_const, pc_out,
                                 &all_org_idx);

  EXPECT_EQ(pc_out->size(), 128);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_circular_org_partial2) {
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(64, 64, base::PointF()));
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  base::PointF center_pt, tmp_pt;
  typedef std::vector<std::pair<int, int>> VectorPair;
  VectorPair all_org_idx;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  for (size_t i = 0; i < 64; i++) {
    for (size_t j = 0; j < 64; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 1.f;
      *(pc_in->at(i, j)) = tmp_pt;
      all_org_idx.push_back(std::make_pair(i, j));
    }
  }
  pc_in->at(0, 0)->x = 0.0f / 0.0f;
  int org_num = 16;
  int smp_ratio = 8;
  float radius = 100.f;
  int velodyne_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingCircularOrgPartial(center_pt, org_num, smp_ratio, radius,
                                 velodyne_model, pc_in_const, pc_out,
                                 &all_org_idx);

  EXPECT_EQ(pc_out->size(), 127);
}

TEST(PointCloudProcessingDownsamplingTest,
     downsampling_rectangle_org_partial1) {
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(128, 128, base::PointF()));
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  base::PointF center_pt, tmp_pt;
  typedef std::vector<std::pair<int, int>> VectorPair;
  VectorPair all_org_idx;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  for (size_t i = 0; i < 126; i++) {
    for (size_t j = 0; j < 126; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 1.f;
      *(pc_in->at(i, j)) = tmp_pt;
      all_org_idx.push_back(std::make_pair(i, j));
    }
  }
  pc_in->at(0, 0)->x = 0.0f / 0.0f;
  int org_num = 16;
  int smp_ratio = 8;
  float front_range = 63.f;
  float side_range = 63.f;
  int velodyne_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingRectangleOrgPartial(org_num, smp_ratio, front_range, side_range,
                                  velodyne_model, pc_in_const, pc_out,
                                  &all_org_idx);
  EXPECT_EQ(pc_out->size(), 127);
}

TEST(PointCloudProcessingDownsamplingTest,
     downsampling_rectangle_org_partial2) {
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(128, 128, base::PointF()));
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  base::PointF center_pt, tmp_pt;
  typedef std::vector<std::pair<int, int>> VectorPair;
  VectorPair all_org_idx;
  center_pt.x = 0.f;
  center_pt.y = 0.f;
  center_pt.z = 0.f;
  for (size_t i = 0; i < 128; i++) {
    for (size_t j = 0; j < 128; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 1.f;
      *(pc_in->at(i, j)) = tmp_pt;
      all_org_idx.push_back(std::make_pair(i, j));
    }
  }
  int org_num = 16;
  int smp_ratio = 8;
  float front_range = 63.f;
  float side_range = 63.f;
  int velodyne_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingRectangleOrgPartial(org_num, smp_ratio, front_range, side_range,
                                  velodyne_model, pc_in_const, pc_out,
                                  &all_org_idx);
  EXPECT_EQ(pc_out->size(), 128);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_rectangle_neighbour1) {
  base::PointF tmp_pt;
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(64, 64, base::PointF()));
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  for (size_t i = 0; i < 64; i++) {
    for (size_t j = 0; j < 64; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 1.f;
      *(pc_in->at(i, j)) = tmp_pt;
    }
  }
  float front_range = 31.f;
  float side_range = 31.f;
  double max_nei = 2.f;
  int velo_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingRectangleNeighbour(front_range, side_range, max_nei, velo_model,
                                 pc_in_const, pc_out);
  EXPECT_EQ(pc_out->size(), 349);
}

TEST(PointCloudProcessingDownsamplingTest, downsampling_rectangle_neighbour2) {
  base::PointF tmp_pt;
  base::PolygonFTypePtr pc_in =
      base::PolygonFTypePtr(new base::PolygonFType(64, 64, base::PointF()));
  base::PolygonFTypePtr pc_out = base::PolygonFTypePtr(new base::PolygonFType);
  for (size_t i = 0; i < 64; i++) {
    for (size_t j = 0; j < 64; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 1.f;
      *(pc_in->at(i, j)) = tmp_pt;
    }
  }
  pc_in->at(0, 0)->x = 0.0f / 0.0f;
  float front_range = 31.f;
  float side_range = 31.f;
  double max_nei = 2.f;
  int velo_model = 64;
  base::PolygonFTypeConstPtr pc_in_const = pc_in;
  DownsamplingRectangleNeighbour(front_range, side_range, max_nei, velo_model,
                                 pc_in_const, pc_out);
  EXPECT_EQ(pc_out->size(), 349);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
