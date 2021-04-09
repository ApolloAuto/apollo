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

#include "modules/perception/common/geometry_util.h"

#include <cfloat>
#include <fstream>
#include <string>
#include <vector>

#include "boost/algorithm/string.hpp"
#include "gtest/gtest.h"

#include "modules/common/log.h"
#include "modules/perception/obstacle/lidar/object_builder/min_box/min_box.h"

namespace apollo {
namespace perception {

using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointDCloud;

namespace {
const double EPSILON = 1e-6;
}

static bool ConstructPointCloud(std::vector<pcl_util::PointCloudPtr>* clouds) {
  std::string pcd_data(
      "modules/perception/data/obstacle_common_test/"
      "QB9178_3_1461381834_1461382134_30651.pcd");
  std::ifstream cluster_ifs(pcd_data.c_str(), std::ifstream::in);
  std::string point_buf;
  while (cluster_ifs.good()) {
    getline(cluster_ifs, point_buf);
    std::vector<std::string> point_strs;
    boost::algorithm::split(point_strs, point_buf,
                            boost::algorithm::is_any_of(" "));
    if (point_strs.size() <= 1 || (point_strs.size() - 1) % 4 != 0) {
      continue;
    }
    std::stringstream ss;
    ss << point_buf;
    int point_num = 0;
    ss >> point_num;
    int exact_point_num = (point_strs.size() - 1) / 4;
    if (point_num != exact_point_num) {
      continue;
    }
    uint64_t intensity;
    pcl_util::PointCloudPtr cluster_cloud(new pcl_util::PointCloud);
    for (int i = 0; i < exact_point_num; ++i) {
      pcl_util::Point p;
      ss >> p.x >> p.y >> p.z >> intensity;
      p.intensity = static_cast<uint8_t>(intensity);
      cluster_cloud->points.push_back(p);
    }
    clouds->push_back(cluster_cloud);
  }
  return true;
}

class GeometryUtilTest : public testing::Test {
 protected:
  GeometryUtilTest() {}
  virtual ~GeometryUtilTest() {}
  void SetUp() {
    ConstructPointCloud(&clouds_);
    ASSERT_EQ(5, clouds_.size());
    Eigen::Vector3d translation(-0.0189, 1.0061, 1);
    Eigen::Vector4d rotation(0.0099, -0.0029, 0.6989, 0.7151);
    TransAffineToMatrix4(translation, rotation, &trans_matrix_);
  }
  void TearDown() {}

 protected:
  std::vector<pcl_util::PointCloudPtr> clouds_;
  Eigen::Matrix4d trans_matrix_;
};

TEST_F(GeometryUtilTest, TransformPointCloud1) {
  pcl_util::PointCloudPtr in_out_cloud(new pcl_util::PointCloud);
  pcl::copyPointCloud(*clouds_[0], *in_out_cloud);
  TransformPointCloud<pcl_util::Point>(trans_matrix_, in_out_cloud);
  EXPECT_NEAR(64.184380, in_out_cloud->at(0).x, EPSILON);
  EXPECT_NEAR(13.708398, in_out_cloud->at(0).y, EPSILON);
  EXPECT_NEAR(1.715922, in_out_cloud->at(0).z, EPSILON);
}

TEST_F(GeometryUtilTest, TransformPointCloud2) {
  pcl_util::Point point_in, point_out;
  point_in.x = 1.0;
  point_in.y = 1.0;
  point_in.z = 1.0;
  TransformPoint(point_in, trans_matrix_, &point_out);
  EXPECT_NEAR(-0.985772, point_out.x, EPSILON);
  EXPECT_NEAR(2.010278, point_out.y, EPSILON);
  EXPECT_NEAR(2.027878, point_out.z, EPSILON);

  pcl_util::PointCloudPtr in_out_cloud(new pcl_util::PointCloud);
  pcl::copyPointCloud(*clouds_[0], *in_out_cloud);
  TransformPointCloud<pcl_util::Point>(trans_matrix_, in_out_cloud);
  EXPECT_NEAR(64.184380, in_out_cloud->at(0).x, EPSILON);
  EXPECT_NEAR(13.708398, in_out_cloud->at(0).y, EPSILON);
  EXPECT_NEAR(1.715922, in_out_cloud->at(0).z, EPSILON);

  pcl_util::PointCloudPtr out_cloud(new pcl_util::PointCloud);
  TransformPointCloud(*clouds_[0], trans_matrix_, out_cloud.get());
  EXPECT_NEAR(64.184380, out_cloud->at(0).x, EPSILON);
  EXPECT_NEAR(13.708398, out_cloud->at(0).y, EPSILON);
  EXPECT_NEAR(1.715922, out_cloud->at(0).z, EPSILON);
}

TEST_F(GeometryUtilTest, TransformPointCloud3) {
  pcl_util::PointCloudPtr in_cloud(new pcl_util::PointCloud);
  for (int i = 0; i < 10; ++i) {
    pcl_util::Point pt;
    pt.x = static_cast<float>(i);
    pt.y = static_cast<float>(i);
    pt.z = static_cast<float>(i);
    pt.intensity = 123.0f;
    in_cloud->push_back(pt);
  }

  pcl_util::PointDCloud out_cloud;
  std::vector<int> indices{0, 2, 5, 9};
  TransformPointCloud(in_cloud, indices, &out_cloud);

  EXPECT_EQ(out_cloud.points.size(), 4);
  EXPECT_NEAR(out_cloud.points[0].x, 0.0, 1e-6);
  EXPECT_NEAR(out_cloud.points[1].y, 2.0, 1e-6);
  EXPECT_NEAR(out_cloud.points[2].z, 5.0, 1e-6);
  EXPECT_EQ(out_cloud.points[3].intensity, 123u);
}

TEST_F(GeometryUtilTest, TransformPointCloud) {
  pcl_util::PointDCloudPtr trans_cloud(new pcl_util::PointDCloud);
  TransformPointCloud(clouds_[0], trans_matrix_, trans_cloud);
  EXPECT_NEAR(64.184377, trans_cloud->at(0).x, EPSILON);
  EXPECT_NEAR(13.708398, trans_cloud->at(0).y, EPSILON);
  EXPECT_NEAR(1.715922, trans_cloud->at(0).z, EPSILON);
}

TEST_F(GeometryUtilTest, GetCloudMinMax3D) {
  pcl_util::PointCloudPtr in_cloud(new pcl_util::PointCloud);
  in_cloud->is_dense = true;
  for (int i = 0; i < 10; ++i) {
    pcl_util::Point pt;
    pt.x = static_cast<float>(i);
    pt.y = static_cast<float>(i);
    pt.z = static_cast<float>(i);
    pt.intensity = 123.0f;
    in_cloud->push_back(pt);
  }
  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  GetCloudMinMax3D<pcl_util::Point>(in_cloud, &min_point, &max_point);
  EXPECT_NEAR(min_point.x(), 0.0, EPSILON);
  EXPECT_NEAR(min_point.y(), 0.0, EPSILON);
  EXPECT_NEAR(min_point.z(), 0.0, EPSILON);
  EXPECT_NEAR(max_point.x(), 9.0, EPSILON);
  EXPECT_NEAR(max_point.y(), 9.0, EPSILON);
  EXPECT_NEAR(max_point.z(), 9.0, EPSILON);

  in_cloud->is_dense = false;
  pcl_util::Point pt;
  pt.x = NAN;
  pt.y = NAN;
  pt.z = NAN;
  in_cloud->push_back(pt);
  GetCloudMinMax3D<pcl_util::Point>(in_cloud, &min_point, &max_point);
  EXPECT_NEAR(min_point.x(), 0.0, EPSILON);
  EXPECT_NEAR(min_point.y(), 0.0, EPSILON);
  EXPECT_NEAR(min_point.z(), 0.0, EPSILON);
  EXPECT_NEAR(max_point.x(), 9.0, EPSILON);
  EXPECT_NEAR(max_point.y(), 9.0, EPSILON);
  EXPECT_NEAR(max_point.z(), 9.0, EPSILON);
}

TEST_F(GeometryUtilTest, ComputeBboxSizeCenter) {
  std::vector<std::shared_ptr<Object>> objects;
  for (size_t i = 0; i < clouds_.size(); ++i) {
    std::shared_ptr<Object> object(new Object);
    object->cloud = clouds_[i];
    objects.push_back(object);
  }
  EXPECT_EQ(5, objects.size());
  ObjectBuilderOptions options;
  MinBoxObjectBuilder min_box_object_builder;
  EXPECT_TRUE(min_box_object_builder.Build(options, &objects));
  Eigen::Vector3d old_dir = objects[0]->direction;
  Eigen::Vector3d old_size = Eigen::Vector3d(
      objects[0]->length, objects[0]->width, objects[0]->height);
  Eigen::Vector3d old_center = objects[0]->center;
  Eigen::Vector3d new_size = old_size;
  Eigen::Vector3d new_center = old_center;
  ComputeBboxSizeCenter<pcl_util::Point>(objects[0]->cloud, old_dir, &new_size,
                                         &new_center);
  EXPECT_NEAR(0.149998, new_size[0], EPSILON);
  EXPECT_NEAR(0.056100, new_size[1], EPSILON);
  EXPECT_NEAR(0.732072, new_size[2], EPSILON);
  EXPECT_NEAR(14.186950, new_center[0], EPSILON);
  EXPECT_NEAR(-63.964300, new_center[1], EPSILON);
  EXPECT_NEAR(0.374468, new_center[2], EPSILON);
}

TEST_F(GeometryUtilTest, GetCloudBarycenter) {
  Eigen::Vector3f barycenter;
  // case 1
  barycenter =
      GetCloudBarycenter<apollo::perception::pcl_util::Point>(clouds_[0])
          .cast<float>();
  EXPECT_NEAR(14.188400, barycenter[0], EPSILON);
  // case 2
  barycenter =
      GetCloudBarycenter<apollo::perception::pcl_util::Point>(clouds_[1])
          .cast<float>();
  EXPECT_NEAR(15.661365, barycenter[0], EPSILON);
  // case 3
  barycenter =
      GetCloudBarycenter<apollo::perception::pcl_util::Point>(clouds_[2])
          .cast<float>();
  EXPECT_NEAR(29.376224, barycenter[0], EPSILON);
  // case 4
  barycenter =
      GetCloudBarycenter<apollo::perception::pcl_util::Point>(clouds_[3])
          .cast<float>();
  EXPECT_NEAR(13.144600, barycenter[0], EPSILON);
  // case 5
  barycenter =
      GetCloudBarycenter<apollo::perception::pcl_util::Point>(clouds_[4])
          .cast<float>();
  EXPECT_NEAR(14.668054, barycenter[0], EPSILON);
}

TEST_F(GeometryUtilTest, TransAffineToMatrix4) {
  Eigen::Vector3d translation(-0.0189, 1.0061, 1);
  Eigen::Vector4d rotation(0.0099, -0.0029, 0.6989, 0.7151);
  Eigen::Matrix4d trans_matrix_;
  TransAffineToMatrix4(translation, rotation, &trans_matrix_);
  AINFO << "trans_matrix: " << trans_matrix_;
}

TEST_F(GeometryUtilTest, ComputeMostConsistentBboxDirection) {
  Eigen::Vector3f previous_dir(1.0, 0.0, 0.0);
  Eigen::Vector3f current_dir(0.0, 1.0, 0.0);
  ComputeMostConsistentBboxDirection(previous_dir, &current_dir);
  EXPECT_NEAR(-1.0, current_dir[0], EPSILON);
  EXPECT_NEAR(0.0, current_dir[1], EPSILON);
  EXPECT_NEAR(0.0, current_dir[2], EPSILON);
}

TEST_F(GeometryUtilTest, VectorCosTheta2dXy) {
  // case 1
  Eigen::Vector3f track_motion_dir(1.0, 1.0, 0.0);
  Eigen::Vector3f anchor_point_shift_dir(1.0, 1.0, 0.0);
  track_motion_dir.normalize();
  anchor_point_shift_dir.normalize();
  double cos_theta =
      VectorCosTheta2dXy(track_motion_dir, anchor_point_shift_dir);
  EXPECT_NEAR(1.0, cos_theta, EPSILON);
  // case 2
  track_motion_dir << 2.0, 2.2, 3.1;
  anchor_point_shift_dir << 1.1, 2.0, 3.5;
  cos_theta = VectorCosTheta2dXy(track_motion_dir, anchor_point_shift_dir);
  track_motion_dir.normalize();
  anchor_point_shift_dir.normalize();
  EXPECT_NEAR(0.972521, cos_theta, EPSILON);
  // case 3
  track_motion_dir << 25.0, 72.2, 3.24;
  anchor_point_shift_dir << 31.1, 98.0, 24.5;
  cos_theta = VectorCosTheta2dXy(track_motion_dir, anchor_point_shift_dir);
  track_motion_dir.normalize();
  anchor_point_shift_dir.normalize();
  EXPECT_NEAR(0.999661, cos_theta, EPSILON);
}

TEST_F(GeometryUtilTest, VectorTheta2dXy) {
  // case 1
  Eigen::Vector3f coord_dir(0, 1.0, 0);
  Eigen::Vector3f dir_velo3(1.0, 0.0, 0.0);
  double theta = VectorCosTheta2dXy(coord_dir, dir_velo3);
  EXPECT_NEAR(0.0, theta, EPSILON);
  // case 2
  coord_dir << 1.0, 0.0, 0.0;
  dir_velo3 << 1.0, 0.0, 0.0;
  theta = VectorCosTheta2dXy(coord_dir, dir_velo3);
  EXPECT_NEAR(1.0, theta, EPSILON);
  // case 3
  coord_dir << 0.3, 0.4, -3.2;
  coord_dir.normalize();
  dir_velo3 << 2.3, -21.4, 4.2;
  dir_velo3.normalize();
  theta = VectorCosTheta2dXy(coord_dir, dir_velo3);
  EXPECT_NEAR(-0.731302, theta, EPSILON);
  // case 4
  coord_dir << 2.0, 3.0, -4.6;
  coord_dir.normalize();
  dir_velo3 << -1.0, 2.4, -3.2;
  dir_velo3.normalize();
  theta = VectorCosTheta2dXy(coord_dir, dir_velo3);
  EXPECT_NEAR(0.554700, theta, EPSILON);
}

}  // namespace perception
}  // namespace apollo
