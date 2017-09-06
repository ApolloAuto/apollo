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

#include "modules/perception/obstacle/common/convex_hullxy.h"

#include <fstream>

#include "gtest/gtest.h"
#include "modules/common/log.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"

namespace apollo {
namespace perception {

using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointDCloud;

static bool ConstructPointCloud(std::vector<pcl_util::PointCloudPtr>* clouds) {
  std::string pcd_data(
      "modules/perception/data/obstacle_common_test/"
      "QB9178_3_1461381834_1461382134_30651.pcd");
  std::ifstream cluster_ifs(pcd_data.c_str(), std::ifstream::in);
  std::string point_buf;
  while (cluster_ifs.good()) {
    getline(cluster_ifs, point_buf);
    std::stringstream ss;
    ss << point_buf;
    int point_num = 0;
    ss >> point_num;
    if (point_num <= 0) {
      continue;
    }
    uint64_t intensity;
    pcl_util::PointCloudPtr cluster_cloud(new pcl_util::PointCloud);
    for (int i = 0; i < point_num; ++i) {
      pcl_util::Point p;
      ss >> p.x >> p.y >> p.z >> intensity;
      p.intensity = static_cast<uint8_t>(intensity);
      cluster_cloud->points.push_back(p);
    }
    clouds->push_back(cluster_cloud);
  }
  return true;
}

TEST(ConvexHull2DXYTest, Reconstruct2dxy) {
  std::vector<pcl_util::PointCloudPtr> clouds;
  ConstructPointCloud(&clouds);
  EXPECT_EQ(5, clouds.size());
  ConvexHull2DXY<pcl_util::Point> convex_hull;
  EXPECT_EQ(convex_hull.getClassName(), "ConvexHull2DXY");
  std::vector<pcl::Vertices> poly_vt;
  PointCloudPtr plane_hull(new PointCloud);
  // case 0
  pcl_util::PointCloudPtr cloud(new pcl_util::PointCloud);
  pcl_util::Point pt;
  pt.x = 1.0;
  pt.y = 0.0;
  pt.z = 0.0;
  cloud->push_back(pt);
  pt.x = 1.0;
  pt.y = 1.0;
  pt.z = 0.0;
  cloud->push_back(pt);
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = 0.0;
  cloud->push_back(pt);
  poly_vt.clear();
  plane_hull->clear();
  convex_hull.setInputCloud(cloud);
  convex_hull.setDimension(2);
  convex_hull.Reconstruct2dxy(plane_hull, &poly_vt);
  EXPECT_EQ(3, poly_vt[0].vertices.size());
  // case 1
  poly_vt.clear();
  plane_hull->clear();
  convex_hull.setInputCloud(clouds[0]);
  convex_hull.setDimension(2);
  convex_hull.Reconstruct2dxy(plane_hull, &poly_vt);
  EXPECT_EQ(1, poly_vt.size());
  EXPECT_EQ(3, poly_vt[0].vertices.size());
  // case 2
  poly_vt.clear();
  plane_hull->clear();
  convex_hull.setInputCloud(clouds[1]);
  convex_hull.setDimension(2);
  convex_hull.Reconstruct2dxy(plane_hull, &poly_vt);
  EXPECT_EQ(1, poly_vt.size());
  EXPECT_EQ(5, poly_vt[0].vertices.size());
  // case 3
  poly_vt.clear();
  plane_hull->clear();
  convex_hull.setInputCloud(clouds[2]);
  convex_hull.setDimension(2);
  convex_hull.Reconstruct2dxy(plane_hull, &poly_vt);
  EXPECT_EQ(1, poly_vt.size());
  EXPECT_EQ(4, poly_vt[0].vertices.size());
  // case 4
  poly_vt.clear();
  plane_hull->clear();
  convex_hull.setInputCloud(clouds[3]);
  convex_hull.setDimension(2);
  convex_hull.Reconstruct2dxy(plane_hull, &poly_vt);
  EXPECT_EQ(1, poly_vt.size());
  EXPECT_EQ(6, poly_vt[0].vertices.size());
  // case 5
  poly_vt.clear();
  plane_hull->clear();
  convex_hull.setInputCloud(clouds[4]);
  convex_hull.setDimension(2);
  convex_hull.Reconstruct2dxy(plane_hull, &poly_vt);
  EXPECT_EQ(1, poly_vt.size());
  EXPECT_EQ(6, poly_vt[0].vertices.size());
}

}  // namespace perception
}  // namespace apollo
