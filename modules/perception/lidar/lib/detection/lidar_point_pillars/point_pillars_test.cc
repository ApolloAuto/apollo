/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file test_point_pillars.cpp
 * @brief unit test file
 * @author Kosuke Murakami
 * @date 2019/02/26
 */

#include <vector>

#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points.h"
#include "modules/perception/tool/benchmark/lidar/util/io_util.h"

namespace apollo {
namespace perception {
namespace lidar {

class TestSuite : public ::testing::Test {
 public:
  TestSuite() {}
  ~TestSuite() {}
};

class TestClass {
 public:
  TestClass(const int MAX_NUM_PILLARS, const int MAX_NUM_POINTS_PER_PILLAR,
            const int GRID_X_SIZE, const int GRID_Y_SIZE, const int GRID_Z_SIZE,
            const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
            const float PILLAR_Z_SIZE, const float MIN_X_RANGE,
            const float MIN_Y_RANGE, const float MIN_Z_RANGE,
            const int NUM_INDS_FOR_SCAN, const int NUM_BOX_CORNERS);
  const int MAX_NUM_PILLARS_;
  const int MAX_NUM_POINTS_PER_PILLAR_;
  const int GRID_X_SIZE_;
  const int GRID_Y_SIZE_;
  const int GRID_Z_SIZE_;
  const float PILLAR_X_SIZE_;
  const float PILLAR_Y_SIZE_;
  const float PILLAR_Z_SIZE_;
  const float MIN_X_RANGE_;
  const float MIN_Y_RANGE_;
  const float MIN_Z_RANGE_;
  const int NUM_INDS_FOR_SCAN_;
  const int NUM_BOX_CORNERS_;

  // Make pointcloud for test
  void makePointsForTest(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr);
  void pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr,
                  float* out_points_array,
                  const float normalizing_factor = 1.0);
  void preprocess(const float* in_points_array, int in_num_points, int* x_coors,
                  int* y_coors, float* num_points_per_pillar, float* pillar_x,
                  float* pillar_y, float* pillar_z, float* pillar_i,
                  float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped,
                  float* pillar_feature_mask, float* sparse_pillar_map,
                  int* host_pillar_count);
  void generateAnchors(float* anchors_px, float* anchors_py, float* anchors_pz,
                       float* anchors_dx, float* anchors_dy, float* anchors_dz,
                       float* anchors_ro);
  void convertAnchors2BoxAnchors(float* anchors_px, float* anchors_py,
                                 float* anchors_dx, float* anchors_dy,
                                 float* box_anchors_min_x,
                                 float* box_anchors_min_y,
                                 float* box_anchors_max_x,
                                 float* box_anchors_max_y);
  void DoInference(const float* in_points_array, const int in_num_points,
                   std::vector<float>* out_detections);

 private:
  std::unique_ptr<PreprocessPoints> preprocess_points_ptr_;
  std::unique_ptr<PointPillars> point_pillars_ptr_;
};

TestClass::TestClass(const int MAX_NUM_PILLARS,
                     const int MAX_NUM_POINTS_PER_PILLAR, const int GRID_X_SIZE,
                     const int GRID_Y_SIZE, const int GRID_Z_SIZE,
                     const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
                     const float PILLAR_Z_SIZE, const float MIN_X_RANGE,
                     const float MIN_Y_RANGE, const float MIN_Z_RANGE,
                     const int NUM_INDS_FOR_SCAN, const int NUM_BOX_CORNERS)
    : MAX_NUM_PILLARS_(MAX_NUM_PILLARS),
      MAX_NUM_POINTS_PER_PILLAR_(MAX_NUM_POINTS_PER_PILLAR),
      GRID_X_SIZE_(GRID_X_SIZE),
      GRID_Y_SIZE_(GRID_Y_SIZE),
      GRID_Z_SIZE_(GRID_Z_SIZE),
      PILLAR_X_SIZE_(PILLAR_X_SIZE),
      PILLAR_Y_SIZE_(PILLAR_Y_SIZE),
      PILLAR_Z_SIZE_(PILLAR_Z_SIZE),
      MIN_X_RANGE_(MIN_X_RANGE),
      MIN_Y_RANGE_(MIN_Y_RANGE),
      MIN_Z_RANGE_(MIN_Z_RANGE),
      NUM_INDS_FOR_SCAN_(NUM_INDS_FOR_SCAN),
      NUM_BOX_CORNERS_(NUM_BOX_CORNERS) {
  preprocess_points_ptr_.reset(new PreprocessPoints(
      MAX_NUM_PILLARS_, MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_, GRID_Y_SIZE_,
      GRID_Z_SIZE_, PILLAR_X_SIZE_, PILLAR_Y_SIZE_, PILLAR_Z_SIZE_,
      MIN_X_RANGE_, MIN_Y_RANGE_, MIN_Z_RANGE_, NUM_INDS_FOR_SCAN_,
      NUM_BOX_CORNERS_));

  //  bool baselink_support=true;
  bool reproduce_result_mode = false;
  float score_threshold = 0.5;
  float nms_overlap_threshold = 0.5;

  point_pillars_ptr_.reset(
      new PointPillars(reproduce_result_mode, score_threshold,
                       nms_overlap_threshold, FLAGS_pfe_onnx_file,
                       FLAGS_rpn_onnx_file));
};

void TestClass::preprocess(const float* in_points_array, int in_num_points,
                           int* x_coors, int* y_coors,
                           float* num_points_per_pillar, float* pillar_x,
                           float* pillar_y, float* pillar_z, float* pillar_i,
                           float* x_coors_for_sub_shaped,
                           float* y_coors_for_sub_shaped,
                           float* pillar_feature_mask, float* sparse_pillar_map,
                           int* host_pillar_count) {
  preprocess_points_ptr_->preprocess(
      in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar,
      pillar_x, pillar_y, pillar_z, pillar_i, x_coors_for_sub_shaped,
      y_coors_for_sub_shaped, pillar_feature_mask, sparse_pillar_map,
      host_pillar_count);
}

void TestClass::pclToArray(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr,
    float* out_points_array, const float normalizing_factor) {
  for (size_t i = 0; i < in_pcl_pc_ptr->size(); ++i) {
    pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
    out_points_array[i * 4 + 0] = point.x;
    out_points_array[i * 4 + 1] = point.y;
    out_points_array[i * 4 + 2] = point.z;
    out_points_array[i * 4 + 3] =
        static_cast<float>(point.intensity / normalizing_factor);
  }
}

void TestClass::makePointsForTest(
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr) {
  pcl::PointXYZI point;
  point.x = 12.9892;
  point.y = -9.98058;
  point.z = 0;
  point.intensity = 4;
  in_pcl_pc_ptr->push_back(point);
  point.x = 11.8697;
  point.y = -11.123;
  point.z = -0.189377;
  point.intensity = 35;
  in_pcl_pc_ptr->push_back(point);
  point.x = 12.489;
  point.y = -9.59703;
  point.z = -2.15565;
  point.intensity = 11;
  in_pcl_pc_ptr->push_back(point);
  point.x = 12.9084;
  point.y = -10.9626;
  point.z = -2.15565;
  point.intensity = 11;
  in_pcl_pc_ptr->push_back(point);
  point.x = 13.8676;
  point.y = -9.61668;
  point.z = 0.0980819;
  point.intensity = 14;
  in_pcl_pc_ptr->push_back(point);
  point.x = 13.5673;
  point.y = -12.9834;
  point.z = 0.21862;
  point.intensity = 1;
  in_pcl_pc_ptr->push_back(point);
  point.x = 13.8213;
  point.y = -10.8529;
  point.z = -1.22883;
  point.intensity = 19;
  in_pcl_pc_ptr->push_back(point);
  point.x = 11.8957;
  point.y = -10.3189;
  point.z = -1.28556;
  point.intensity = 13;
  in_pcl_pc_ptr->push_back(point);
}

void TestClass::generateAnchors(float* anchors_px, float* anchors_py,
                                float* anchors_pz, float* anchors_dx,
                                float* anchors_dy, float* anchors_dz,
                                float* anchors_ro) {
  return point_pillars_ptr_->generateAnchors(anchors_px, anchors_py, anchors_pz,
                                             anchors_dx, anchors_dy, anchors_dz,
                                             anchors_ro);
}

void TestClass::convertAnchors2BoxAnchors(float* anchors_px, float* anchors_py,
                                          float* anchors_dx, float* anchors_dy,
                                          float* box_anchors_min_x,
                                          float* box_anchors_min_y,
                                          float* box_anchors_max_x,
                                          float* box_anchors_max_y) {
  return point_pillars_ptr_->convertAnchors2BoxAnchors(
      anchors_px, anchors_py, anchors_dx, anchors_dy, box_anchors_min_x,
      box_anchors_min_y, box_anchors_max_x, box_anchors_max_y);
}

void TestClass::DoInference(const float* in_points_array,
                            const int in_num_points,
                            std::vector<float>* out_detections) {
  return point_pillars_ptr_->doInference(in_points_array, in_num_points,
                                         out_detections);
}

TEST(TestSuite, CheckPreprocessPointsCPU) {
  const int MAX_NUM_PILLARS = 12000;
  const int MAX_NUM_POINTS_PER_PILLAR = 100;
  const int GRID_X_SIZE = 432;
  const int GRID_Y_SIZE = 496;
  const int GRID_Z_SIZE = 1;
  const float PILLAR_X_SIZE = 0.16;
  const float PILLAR_Y_SIZE = 0.16;
  const float PILLAR_Z_SIZE = 4.0;
  const float MIN_X_RANGE = 0;
  const float MIN_Y_RANGE = -39.68;
  const float MIN_Z_RANGE = -3.0;
  const int NUM_INDS_FOR_SCAN = 512;
  const int NUM_BOX_CORNERS = 4;
  TestClass test_obj(MAX_NUM_PILLARS, MAX_NUM_POINTS_PER_PILLAR, GRID_X_SIZE,
                     GRID_Y_SIZE, GRID_Z_SIZE, PILLAR_X_SIZE, PILLAR_Y_SIZE,
                     PILLAR_Z_SIZE, MIN_X_RANGE, MIN_Y_RANGE, MIN_Z_RANGE,
                     NUM_INDS_FOR_SCAN, NUM_BOX_CORNERS);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  test_obj.makePointsForTest(pcl_pc_ptr);

  float* points_array = new float[pcl_pc_ptr->size() * 4];
  test_obj.pclToArray(pcl_pc_ptr, points_array);

  int x_coors[MAX_NUM_PILLARS];
  x_coors[0] = 0;
  int y_coors[MAX_NUM_PILLARS];
  y_coors[0] = 0;
  float num_points_per_pillar[MAX_NUM_PILLARS];
  num_points_per_pillar[0] = 0;
  float* pillar_x = new float[test_obj.MAX_NUM_PILLARS_ *
                              test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_y = new float[test_obj.MAX_NUM_PILLARS_ *
                              test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_z = new float[test_obj.MAX_NUM_PILLARS_ *
                              test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_i = new float[test_obj.MAX_NUM_PILLARS_ *
                              test_obj.MAX_NUM_POINTS_PER_PILLAR_];

  float* x_coors_for_sub_shaped =
      new float[test_obj.MAX_NUM_PILLARS_ *
                test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* y_coors_for_sub_shaped =
      new float[test_obj.MAX_NUM_PILLARS_ *
                test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_feature_mask = new float[test_obj.MAX_NUM_PILLARS_ *
                                         test_obj.MAX_NUM_POINTS_PER_PILLAR_];

  float* sparse_pillar_map = new float[512 * 512];

  int host_pillar_count[1] = {0};
  test_obj.preprocess(points_array, pcl_pc_ptr->size(), x_coors, y_coors,
                      num_points_per_pillar, pillar_x, pillar_y, pillar_z,
                      pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped,
                      pillar_feature_mask, sparse_pillar_map,
                      host_pillar_count);
  EXPECT_EQ(1, num_points_per_pillar[0]);
  EXPECT_FLOAT_EQ(12.9892, pillar_x[0]);
  EXPECT_EQ(74, x_coors[1]);
  EXPECT_EQ(178, y_coors[1]);
  EXPECT_EQ(1, sparse_pillar_map[178 * 512 + 74]);
  EXPECT_EQ(8, host_pillar_count[0]);
  delete[] points_array;
  delete[] pillar_x;
  delete[] pillar_y;
  delete[] pillar_z;
  delete[] pillar_i;
  delete[] x_coors_for_sub_shaped;
  delete[] y_coors_for_sub_shaped;
  delete[] pillar_feature_mask;
  delete[] sparse_pillar_map;
}

TEST(TestSuite, CheckGenerateAnchors) {
  const int MAX_NUM_PILLARS = 12000;
  const int MAX_NUM_POINTS_PER_PILLAR = 100;
  const int GRID_X_SIZE = 432;
  const int GRID_Y_SIZE = 496;
  const int GRID_Z_SIZE = 1;
  const float PILLAR_X_SIZE = 0.16;
  const float PILLAR_Y_SIZE = 0.16;
  const float PILLAR_Z_SIZE = 4.0;
  const float MIN_X_RANGE = 0;
  const float MIN_Y_RANGE = -39.68;
  const float MIN_Z_RANGE = -3.0;
  const int NUM_INDS_FOR_SCAN = 512;
  const int NUM_BOX_CORNERS = 4;
  TestClass test_obj(MAX_NUM_PILLARS, MAX_NUM_POINTS_PER_PILLAR, GRID_X_SIZE,
                     GRID_Y_SIZE, GRID_Z_SIZE, PILLAR_X_SIZE, PILLAR_Y_SIZE,
                     PILLAR_Z_SIZE, MIN_X_RANGE, MIN_Y_RANGE, MIN_Z_RANGE,
                     NUM_INDS_FOR_SCAN, NUM_BOX_CORNERS);

  const int NUM_ANCHOR = 432 * 0.5 * 496 * 0.5 * 2;
  float* anchors_px = new float[NUM_ANCHOR];
  float* anchors_py = new float[NUM_ANCHOR];
  float* anchors_pz = new float[NUM_ANCHOR];
  float* anchors_dx = new float[NUM_ANCHOR];
  float* anchors_dy = new float[NUM_ANCHOR];
  float* anchors_dz = new float[NUM_ANCHOR];
  float* anchors_ro = new float[NUM_ANCHOR];
  test_obj.generateAnchors(anchors_px, anchors_py, anchors_pz, anchors_dx,
                           anchors_dy, anchors_dz, anchors_ro);

  EXPECT_NEAR(0.48, anchors_px[3], 0.001);
  EXPECT_NEAR(-39.52, anchors_py[109], 0.001);
  EXPECT_NEAR(-1.73, anchors_pz[76], 0.001);
  EXPECT_NEAR(1.6, anchors_dx[338], 0.001);
  EXPECT_NEAR(3.9, anchors_dy[22], 0.001);
  EXPECT_NEAR(1.56, anchors_dz[993], 0.001);
  EXPECT_NEAR(1.5708, anchors_ro[1765], 0.001);

  delete[] anchors_px;
  delete[] anchors_py;
  delete[] anchors_pz;
  delete[] anchors_dx;
  delete[] anchors_dy;
  delete[] anchors_dz;
  delete[] anchors_ro;
}

TEST(TestSuite, CheckGenerateBoxAnchors) {
  const int MAX_NUM_PILLARS = 12000;
  const int MAX_NUM_POINTS_PER_PILLAR = 100;
  const int GRID_X_SIZE = 432;
  const int GRID_Y_SIZE = 496;
  const int GRID_Z_SIZE = 1;
  const float PILLAR_X_SIZE = 0.16;
  const float PILLAR_Y_SIZE = 0.16;
  const float PILLAR_Z_SIZE = 4.0;
  const float MIN_X_RANGE = 0;
  const float MIN_Y_RANGE = -39.68;
  const float MIN_Z_RANGE = -3.0;
  const int NUM_INDS_FOR_SCAN = 512;
  const int NUM_BOX_CORNERS = 4;
  TestClass test_obj(MAX_NUM_PILLARS, MAX_NUM_POINTS_PER_PILLAR, GRID_X_SIZE,
                     GRID_Y_SIZE, GRID_Z_SIZE, PILLAR_X_SIZE, PILLAR_Y_SIZE,
                     PILLAR_Z_SIZE, MIN_X_RANGE, MIN_Y_RANGE, MIN_Z_RANGE,
                     NUM_INDS_FOR_SCAN, NUM_BOX_CORNERS);

  const int NUM_ANCHOR = 432 * 0.5 * 496 * 0.5 * 2;

  float* anchors_px = new float[NUM_ANCHOR];
  float* anchors_py = new float[NUM_ANCHOR];
  float* anchors_pz = new float[NUM_ANCHOR];
  float* anchors_dx = new float[NUM_ANCHOR];
  float* anchors_dy = new float[NUM_ANCHOR];
  float* anchors_dz = new float[NUM_ANCHOR];
  float* anchors_ro = new float[NUM_ANCHOR];
  float* box_anchors_min_x = new float[NUM_ANCHOR];
  float* box_anchors_min_y = new float[NUM_ANCHOR];
  float* box_anchors_max_x = new float[NUM_ANCHOR];
  float* box_anchors_max_y = new float[NUM_ANCHOR];
  test_obj.generateAnchors(anchors_px, anchors_py, anchors_pz, anchors_dx,
                           anchors_dy, anchors_dz, anchors_ro);
  test_obj.convertAnchors2BoxAnchors(
      anchors_px, anchors_py, anchors_dx, anchors_dy, box_anchors_min_x,
      box_anchors_min_y, box_anchors_max_x, box_anchors_max_y);

  EXPECT_NEAR(53.25, box_anchors_min_x[345], 0.001);
  EXPECT_NEAR(-41.47, box_anchors_min_y[22], 0.001);
  EXPECT_NEAR(38.4, box_anchors_max_x[1098], 0.001);
  EXPECT_NEAR(-38.4, box_anchors_max_y[675], 0.001);

  delete[] anchors_px;
  delete[] anchors_py;
  delete[] anchors_pz;
  delete[] anchors_dx;
  delete[] anchors_dy;
  delete[] anchors_dz;
  delete[] anchors_ro;
  delete[] box_anchors_min_x;
  delete[] box_anchors_min_y;
  delete[] box_anchors_max_x;
  delete[] box_anchors_max_y;
}

TEST(TestSuite, CheckDoInference) {
  const int MAX_NUM_PILLARS = 12000;
  const int MAX_NUM_POINTS_PER_PILLAR = 100;
  const int GRID_X_SIZE = 432;
  const int GRID_Y_SIZE = 496;
  const int GRID_Z_SIZE = 1;
  const float PILLAR_X_SIZE = 0.16;
  const float PILLAR_Y_SIZE = 0.16;
  const float PILLAR_Z_SIZE = 4.0;
  const float MIN_X_RANGE = 0;
  const float MIN_Y_RANGE = -39.68;
  const float MIN_Z_RANGE = -3.0;
  const int NUM_INDS_FOR_SCAN = 512;
  const int NUM_BOX_CORNERS = 4;
  const int OUTPUT_NUM_BOX_FEATURE = 7;
  const float NORMALIZING_FACTOR = 255.0;
  TestClass test_obj(MAX_NUM_PILLARS, MAX_NUM_POINTS_PER_PILLAR, GRID_X_SIZE,
                     GRID_Y_SIZE, GRID_Z_SIZE, PILLAR_X_SIZE, PILLAR_Y_SIZE,
                     PILLAR_Z_SIZE, MIN_X_RANGE, MIN_Y_RANGE, MIN_Z_RANGE,
                     NUM_INDS_FOR_SCAN, NUM_BOX_CORNERS);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  apollo::perception::benchmark::PointCloudPtr org_cloud_ptr(
      new pcl::PointCloud<apollo::perception::benchmark::PointXYZIL>);
  std::string file_name =
      "/apollo/modules/perception/testdata/lidar/app/data/perception/"
      "lidar/files/0001_00.pcd";

  bool ret = apollo::perception::benchmark::load_pcl_pcds_xyzit(file_name,
                                                                org_cloud_ptr);
  ASSERT_TRUE(ret) << "Failed to load pcd file: " << file_name;

  for (size_t i = 0; i < org_cloud_ptr->size(); ++i) {
    pcl::PointXYZI point;
    point.x = org_cloud_ptr->at(i).x;
    point.y = org_cloud_ptr->at(i).y;
    point.z = org_cloud_ptr->at(i).z;
    point.intensity = org_cloud_ptr->at(i).intensity;
    pcl_pc_ptr->push_back(point);
  }

  float* points_array = new float[pcl_pc_ptr->size() * 4];
  test_obj.pclToArray(pcl_pc_ptr, points_array, NORMALIZING_FACTOR);

  std::vector<float>* out_detections(new std::vector<float>());
  test_obj.DoInference(points_array, pcl_pc_ptr->size(), out_detections);

  int num_objects = out_detections->size() / OUTPUT_NUM_BOX_FEATURE;
  EXPECT_GE(num_objects, 4);

  for (int j = 0; j < num_objects; ++j) {
    float x = out_detections->at(j * OUTPUT_NUM_BOX_FEATURE + 0);
    float y = out_detections->at(j * OUTPUT_NUM_BOX_FEATURE + 1);
    float z = out_detections->at(j * OUTPUT_NUM_BOX_FEATURE + 2);
    float dx = out_detections->at(j * OUTPUT_NUM_BOX_FEATURE + 4);
    float dy = out_detections->at(j * OUTPUT_NUM_BOX_FEATURE + 3);
    float dz = out_detections->at(j * OUTPUT_NUM_BOX_FEATURE + 5);
    float yaw = out_detections->at(j * OUTPUT_NUM_BOX_FEATURE + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    yaw = -yaw;

    std::cout << "object id: " << j << ", x: " << x << ", y: " << y
              << ", z: " << z << ", dx: " << dx << ", dy: " << dy
              << ", dz: " << dz << ", yaw: " << yaw << std::endl;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
