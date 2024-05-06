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
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_cluster_list.h"

#include "gtest/gtest.h"

#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_label_image.h"

namespace apollo {
namespace perception {
namespace lidar {

TEST(SppClusterTest, spp_cluster_test) {
  SppLabelImage image;
  SppLabelImage& const_image = image;
  image.Init(5, 5);
  EXPECT_EQ(image.width_, 5);
  EXPECT_EQ(image.height_, 5);
  /* test labels
   * 0 0 0 0 3
   * 0 1 0 3 3
   * 0 1 0 0 0
   * 0 1 0 2 0
   * 0 0 0 2 0
   * */
  image.AddPixelSample(0, 1, 1);
  image.AddPixelSample(0, 1, 2);
  image.AddPixelSample(0, 1, 3);

  image.AddPixelSample(1, 3, 3);
  image.AddPixelSample(1, 3, 4);

  image.AddPixelSample(2, 4, 0);
  image.AddPixelSample(2, 4, 1);
  image.AddPixelSample(2, 3, 1);

  image.ProjectClusterToSppLabelImage();
  EXPECT_EQ(image.GetClusterNum(), 3);
  EXPECT_EQ(image.GetClusters().size(), 3);
  EXPECT_EQ(const_image.GetClusters().size(), 3);
  EXPECT_EQ(image.GetCluster(0)->pixels.size(), 3);
  EXPECT_EQ(image.GetCluster(1)->pixels.size(), 2);
  EXPECT_EQ(image.GetCluster(2)->pixels.size(), 3);
  EXPECT_EQ(const_image.GetCluster(2)->pixels.size(), 3);
  image.CollectClusterFromSppLabelImage();
  EXPECT_EQ(image[1][1], 1);
  EXPECT_EQ(const_image[1][1], 1);
  EXPECT_NE(image.GetSppLabelImage(), nullptr);
  EXPECT_NE(const_image.GetSppLabelImage(), nullptr);

  float confidence_map[] = {0.1f, 0.1f, 0.1f, 0.1f, 0.3f, 0.1f, 0.6f,
                            0.1f, 0.3f, 0.3f, 0.1f, 0.6f, 0.1f, 0.1f,
                            0.1f, 0.1f, 0.6f, 0.1f, 0.2f, 0.1f, 0.1f,
                            0.1f, 0.1f, 0.2f, 0.1f};

  image.FilterClusters(confidence_map, 0.5f);
  EXPECT_EQ(image.GetClusterNum(), 1);

  float class_map[] = {
      0.1f, 0.1f, 0.1f, 0.1f, 0.3f, 0.1f, 0.2f, 0.1f, 0.3f, 0.3f,
      0.1f, 0.3f, 0.1f, 0.1f, 0.1f, 0.1f, 0.4f, 0.1f, 0.2f, 0.1f,
      0.1f, 0.1f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.3f,
      0.1f, 0.8f, 0.1f, 0.3f, 0.3f, 0.1f, 0.7f, 0.1f, 0.1f, 0.1f,
      0.1f, 0.6f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f, 0.2f, 0.1f,
  };
  image.CalculateClusterClass(class_map, 2);
  EXPECT_EQ(static_cast<size_t>(image.GetCluster(0)->type), 1);

  float heading_map[] = {
      0.1f, 0.1f, 0.1f, 0.1f, 0.3f, 0.1f, 1.0f, 0.1f, 0.3f, 0.3f,
      0.1f, 1.0f, 0.1f, 0.1f, 0.1f, 0.1f, 1.0f, 0.1f, 0.2f, 0.1f,
      0.1f, 0.1f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.3f,
      0.1f, 0.0f, 0.1f, 0.3f, 0.3f, 0.1f, 0.0f, 0.1f, 0.1f, 0.1f,
      0.1f, 0.0f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f, 0.2f, 0.1f,
  };
  image.CalculateClusterHeading(heading_map);
  EXPECT_NEAR(image.GetCluster(0)->yaw, 0.f, 1e-9);

  float top_z_map[] = {0.1f, 0.1f, 0.1f, 0.1f, 0.3f, 0.1f, 0.5f, 0.1f, 0.3f,
                       0.3f, 0.1f, 0.6f, 0.1f, 0.1f, 0.1f, 0.1f, 0.7f, 0.1f,
                       0.2f, 0.1f, 0.1f, 0.1f, 0.1f, 0.2f, 0.1f};

  image.CalculateClusterTopZ(top_z_map);
  EXPECT_NEAR(image.GetCluster(0)->top_z, 0.6f, 1e-4);

  SppClusterList list;
  const SppClusterList& const_list = list;
  list.Init(10);
  EXPECT_EQ(list.size(), 10);
  EXPECT_EQ(const_list.size(), 10);
  EXPECT_EQ(list.clusters().size(), 10);
  EXPECT_EQ(const_list.clusters().size(), 10);
  list.resize(5);
  EXPECT_EQ(list.size(), 5);
  list.Reset();
  EXPECT_EQ(list.size(), 0);
  base::PointF point;
  list.AddPointSample(0, point, 0.5f, 0);
  list.AddPointSample(1, point, 0.5f, 1);
  list.AddPointSample(2, point, 0.5f, 2);
  list.AddPointSample(0, point, 0.5f, 3);
  EXPECT_EQ(list.size(), 3);
  EXPECT_EQ(list[0]->points.size(), 2);
  EXPECT_EQ(list[1]->points.size(), 1);
  EXPECT_EQ(list[2]->points.size(), 1);
  EXPECT_EQ(const_list[0]->points.size(), 2);
  SppClusterList list2;
  list2.Init(2);
  list.Merge(&list2);
  EXPECT_EQ(list.size(), 5);
  list.RemoveEmptyClusters();
  EXPECT_EQ(list.size(), 3);
  list.EraseCluster(10);
  EXPECT_EQ(list.size(), 3);
  list.EraseCluster(0);
  EXPECT_EQ(list[0]->points.size(), 1);

  list.Reset();
  list.resize(1);
  point.x = 0.f;
  point.y = 0.f;
  point.z = -0.1f;
  list.AddPointSample(0, point, point.z, 0);
  point.z = 0.1f;
  list.AddPointSample(0, point, point.z, 1);
  point.z = 0.2f;
  list.AddPointSample(0, point, point.z, 2);
  point.z = 0.3f;
  list.AddPointSample(0, point, point.z, 3);
  point.z = 0.4f;
  list.AddPointSample(0, point, point.z, 4);
  point.z = 1.0f;
  list.AddPointSample(0, point, point.z, 5);
  point.z = 1.1f;
  list.AddPointSample(0, point, point.z, 6);

  EXPECT_EQ(list.HeightCut(0.5, 0), 1);
  EXPECT_EQ(list[0]->points.size(), 5);

  list.Reset();
  list.resize(1);
  point.x = 0.f;
  point.y = 0.f;
  point.z = 0.1f;
  list.AddPointSample(0, point, point.z, 0);
  point.z = 0.1f;
  list.AddPointSample(0, point, point.z, 1);
  point.z = 0.9f;
  list.AddPointSample(0, point, point.z, 2);
  point.z = 0.9f;
  list.AddPointSample(0, point, point.z, 3);
  point.z = 1.8f;
  list.AddPointSample(0, point, point.z, 4);
  point.z = 1.8f;
  list.AddPointSample(0, point, point.z, 5);
  point.z = 1.9f;
  list.AddPointSample(0, point, point.z, 6);

  EXPECT_EQ(list.HeightCut(0.5, 0), 0);
  EXPECT_EQ(list[0]->points.size(), 7);
  EXPECT_EQ(list[0]->points.size(), list[0]->point_ids.size());

  list = image;
  EXPECT_EQ(list.size(), 1);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
