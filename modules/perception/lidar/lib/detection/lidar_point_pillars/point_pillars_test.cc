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
  TestClass();
  TestClass(const int num_class, const int max_num_pillars,
            const int max_num_points_per_pillar, const int num_point_feature,
            const int grid_x_size, const int grid_y_size, const int grid_z_size,
            const float pillar_x_size, const float pillar_y_size,
            const float pillar_z_size, const float min_x_range,
            const float min_y_range, const float min_z_range,
            const int num_inds_for_scan, const int num_threads);
  const int num_class;
  const int max_num_pillars;
  const int max_num_points_per_pillar;
  const int num_point_feature;
  const int grid_x_size;
  const int grid_y_size;
  const int grid_z_size;
  const float pillar_x_size;
  const float pillar_y_size;
  const float pillar_z_size;
  const float min_x_range;
  const float min_y_range;
  const float min_z_range;
  const int num_inds_for_scan;
  const int num_threads;

  // Make pointcloud for test
  void MakePointsForTest(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr);
  void PclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr,
                  float* out_points_array,
                  const float normalizing_factor = 1.0);
  void PclXYZITToArray(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr,
      float* out_points_array, const float normalizing_factor = 1.0);
  void Preprocess(const float* in_points_array, int in_num_points, int* x_coors,
                  int* y_coors, float* num_points_per_pillar,
                  float* pillar_point_feature, float* pillar_coors,
                  float* sparse_pillar_map, int* host_pillar_count);
  void PreprocessGPU(const float* in_points_array, int in_num_points,
                     int* x_coors, int* y_coors, float* num_points_per_pillar,
                     float* pillar_point_feature, float* pillar_coors,
                     int* sparse_pillar_map, int* host_pillar_count);
  void GenerateAnchors(float* anchors_px, float* anchors_py, float* anchors_pz,
                       float* anchors_dx, float* anchors_dy, float* anchors_dz,
                       float* anchors_ro);
  void ConvertAnchors2BoxAnchors(float* anchors_px, float* anchors_py,
                                 float* box_anchors_min_x,
                                 float* box_anchors_min_y,
                                 float* box_anchors_max_x,
                                 float* box_anchors_max_y);
  void DoInference(const float* in_points_array, const int in_num_points,
                   std::vector<float>* out_detections,
                   std::vector<int>* out_labels);

 private:
  std::unique_ptr<PreprocessPoints> preprocess_points_ptr_;
  std::unique_ptr<PreprocessPointsCuda> preprocess_points_cuda_ptr_;
  std::unique_ptr<PointPillars> point_pillars_ptr_;
};

TestClass::TestClass()
    : num_class(3),
      max_num_pillars(12000),
      max_num_points_per_pillar(100),
      num_point_feature(4),
      grid_x_size(280),
      grid_y_size(320),
      grid_z_size(1),
      pillar_x_size(0.25),
      pillar_y_size(0.25),
      pillar_z_size(4.0),
      min_x_range(0),
      min_y_range(-40.0),
      min_z_range(-3.0),
      num_inds_for_scan(512),
      num_threads(64) {
  preprocess_points_ptr_.reset(new PreprocessPoints(
      max_num_pillars, max_num_points_per_pillar, num_point_feature,
      grid_x_size, grid_y_size, grid_z_size, pillar_x_size, pillar_y_size,
      pillar_z_size, min_x_range, min_y_range, min_z_range, num_inds_for_scan));
  preprocess_points_cuda_ptr_.reset(new PreprocessPointsCuda(
      num_threads, max_num_pillars, max_num_points_per_pillar,
      num_point_feature, num_inds_for_scan, grid_x_size, grid_y_size,
      grid_z_size, pillar_x_size, pillar_y_size, pillar_z_size, min_x_range,
      min_y_range, min_z_range));

  bool reproduce_result_mode = false;
  float score_threshold = 0.5;
  float nms_overlap_threshold = 0.5;

  point_pillars_ptr_.reset(
      new PointPillars(reproduce_result_mode, score_threshold,
                       nms_overlap_threshold, FLAGS_pfe_torch_file,
                       FLAGS_scattered_torch_file, FLAGS_backbone_torch_file,
                       FLAGS_fpn_torch_file, FLAGS_bbox_head_torch_file));
}

TestClass::TestClass(const int num_class, const int max_num_pillars,
                     const int max_num_points_per_pillar,
                     const int num_point_feature, const int grid_x_size,
                     const int grid_y_size, const int grid_z_size,
                     const float pillar_x_size, const float pillar_y_size,
                     const float pillar_z_size, const float min_x_range,
                     const float min_y_range, const float min_z_range,
                     const int num_inds_for_scan, const int num_threads)
    : num_class(num_class),
      max_num_pillars(max_num_pillars),
      max_num_points_per_pillar(max_num_points_per_pillar),
      num_point_feature(num_point_feature),
      grid_x_size(grid_x_size),
      grid_y_size(grid_y_size),
      grid_z_size(grid_z_size),
      pillar_x_size(pillar_x_size),
      pillar_y_size(pillar_y_size),
      pillar_z_size(pillar_z_size),
      min_x_range(min_x_range),
      min_y_range(min_y_range),
      min_z_range(min_z_range),
      num_inds_for_scan(num_inds_for_scan),
      num_threads(num_threads) {
  preprocess_points_ptr_.reset(new PreprocessPoints(
      max_num_pillars, max_num_points_per_pillar, num_point_feature,
      grid_x_size, grid_y_size, grid_z_size, pillar_x_size, pillar_y_size,
      pillar_z_size, min_x_range, min_y_range, min_z_range, num_inds_for_scan));
  preprocess_points_cuda_ptr_.reset(new PreprocessPointsCuda(
      num_threads, max_num_pillars, max_num_points_per_pillar,
      num_point_feature, num_inds_for_scan, grid_x_size, grid_y_size,
      grid_z_size, pillar_x_size, pillar_y_size, pillar_z_size, min_x_range,
      min_y_range, min_z_range));

  bool reproduce_result_mode = false;
  float score_threshold = 0.5;
  float nms_overlap_threshold = 0.5;

  point_pillars_ptr_.reset(
      new PointPillars(reproduce_result_mode, score_threshold,
                       nms_overlap_threshold, FLAGS_pfe_torch_file,
                       FLAGS_scattered_torch_file, FLAGS_backbone_torch_file,
                       FLAGS_fpn_torch_file, FLAGS_bbox_head_torch_file));
}

void TestClass::Preprocess(const float* in_points_array, int in_num_points,
                           int* x_coors, int* y_coors,
                           float* num_points_per_pillar,
                           float* pillar_point_feature, float* pillar_coors,
                           float* sparse_pillar_map, int* host_pillar_count) {
  preprocess_points_ptr_->Preprocess(
      in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar,
      pillar_point_feature, pillar_coors, sparse_pillar_map, host_pillar_count);
}

void TestClass::PreprocessGPU(const float* in_points_array, int in_num_points,
                              int* x_coors, int* y_coors,
                              float* num_points_per_pillar,
                              float* pillar_point_feature, float* pillar_coors,
                              int* sparse_pillar_map, int* host_pillar_count) {
  preprocess_points_cuda_ptr_->DoPreprocessPointsCuda(
      in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar,
      pillar_point_feature, pillar_coors, sparse_pillar_map, host_pillar_count);
}

void TestClass::PclToArray(
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

void TestClass::PclXYZITToArray(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr,
    float* out_points_array, const float normalizing_factor) {
  for (size_t i = 0; i < in_pcl_pc_ptr->size(); ++i) {
    pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
    out_points_array[i * 5 + 0] = point.x;
    out_points_array[i * 5 + 1] = point.y;
    out_points_array[i * 5 + 2] = point.z;
    out_points_array[i * 5 + 3] =
        static_cast<float>(point.intensity / normalizing_factor);
    out_points_array[i * 5 + 4] = 0;
  }
}

void TestClass::MakePointsForTest(
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

void TestClass::GenerateAnchors(float* anchors_px, float* anchors_py,
                                float* anchors_pz, float* anchors_dx,
                                float* anchors_dy, float* anchors_dz,
                                float* anchors_ro) {
  return point_pillars_ptr_->GenerateAnchors(anchors_px, anchors_py, anchors_pz,
                                             anchors_dx, anchors_dy, anchors_dz,
                                             anchors_ro);
}

void TestClass::ConvertAnchors2BoxAnchors(float* anchors_px, float* anchors_py,
                                          float* box_anchors_min_x,
                                          float* box_anchors_min_y,
                                          float* box_anchors_max_x,
                                          float* box_anchors_max_y) {
  return point_pillars_ptr_->ConvertAnchors2BoxAnchors(
      anchors_px, anchors_py, box_anchors_min_x, box_anchors_min_y,
      box_anchors_max_x, box_anchors_max_y);
}

void TestClass::DoInference(const float* in_points_array,
                            const int in_num_points,
                            std::vector<float>* out_detections,
                            std::vector<int>* out_labels) {
  return point_pillars_ptr_->DoInference(in_points_array, in_num_points,
                                         out_detections, out_labels);
}
/*
TEST(TestSuite, CheckPreprocessPointsCPU) {
  const int kNumClass = 1;
  const int kMaxNumPillars = 12000;
  const int kMaxNumPointsPerPillar = 100;
  const int kNumPointFeature = 4;
  const int kGridXSize = 432;
  const int kGridYSize = 496;
  const int kGridZSize = 1;
  const float kPillarXSize = 0.16;
  const float kPillarYSize = 0.16;
  const float kPillarZSize = 4.0;
  const float kMinXRange = 0;
  const float kMinYRange = -39.68;
  const float kMinZRange = -3.0;
  const int kNumIndsForScan = 512;
  const int kNumThreads = 64;
  TestClass test_obj(kNumClass, kMaxNumPillars, kMaxNumPointsPerPillar,
                     kNumPointFeature, kGridXSize, kGridYSize, kGridZSize,
                     kPillarXSize, kPillarYSize, kPillarZSize, kMinXRange,
                     kMinYRange, kMinZRange, kNumIndsForScan, kNumThreads);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  test_obj.MakePointsForTest(pcl_pc_ptr);

  float* points_array = new float[pcl_pc_ptr->size() * 4];
  test_obj.PclToArray(pcl_pc_ptr, points_array);

  int x_coors[kMaxNumPillars] = {};
  int y_coors[kMaxNumPillars] = {};
  float num_points_per_pillar[kMaxNumPillars] = {};

  float* pillar_point_feature =
      new float[test_obj.max_num_pillars * test_obj.max_num_points_per_pillar *
                test_obj.num_point_feature];
  float* pillar_coors = new float[test_obj.max_num_pillars * 4];
  float* sparse_pillar_map = new float[kNumIndsForScan * kNumIndsForScan];

  int host_pillar_count[1] = {0};
  test_obj.Preprocess(points_array, pcl_pc_ptr->size(), x_coors, y_coors,
                      num_points_per_pillar, pillar_point_feature, pillar_coors,
                      sparse_pillar_map, host_pillar_count);
  EXPECT_EQ(1, num_points_per_pillar[0]);
  EXPECT_FLOAT_EQ(12.9892, pillar_point_feature[0]);
  EXPECT_EQ(74, x_coors[1]);
  EXPECT_EQ(178, y_coors[1]);
  EXPECT_EQ(1, sparse_pillar_map[178 * 512 + 74]);
  EXPECT_EQ(8, host_pillar_count[0]);

  delete[] points_array;
  delete[] pillar_point_feature;
  delete[] pillar_coors;
  delete[] sparse_pillar_map;
}

TEST(TestSuite, CheckPreprocessGPU) {
  const int kNumClass = 1;
  const int kMaxNumPillars = 12000;
  const int kMaxNumPointsPerPillar = 100;
  const int kNumPointFeature = 4;
  const int kGridXSize = 432;
  const int kGridYSize = 496;
  const int kGridZSize = 1;
  const float kPillarXSize = 0.16;
  const float kPillarYSize = 0.16;
  const float kPillarZSize = 4.0;
  const float kMinXRange = 0;
  const float kMinYRange = -39.68;
  const float kMinZRange = -3.0;
  const int kNumIndsForScan = 512;
  const int kNumThreads = 64;
  const float kNormalizingFactor = 255.0;
  TestClass test_obj(kNumClass, kMaxNumPillars, kMaxNumPointsPerPillar,
                     kNumPointFeature, kGridXSize, kGridYSize, kGridZSize,
                     kPillarXSize, kPillarYSize, kPillarZSize, kMinXRange,
                     kMinYRange, kMinZRange, kNumIndsForScan, kNumThreads);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  apollo::perception::benchmark::PointCloudPtr org_cloud_ptr(
      new pcl::PointCloud<apollo::perception::benchmark::PointXYZIL>);
  std::string file_name =
      "/apollo/modules/perception/testdata/lidar/app/data/0001_00.pcd";

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
  int in_num_points = pcl_pc_ptr->size();
  float* points_array = new float[pcl_pc_ptr->size() * 4];
  test_obj.PclToArray(pcl_pc_ptr, points_array, kNormalizingFactor);

  float* dev_points;
  int* dev_x_coors;
  int* dev_y_coors;
  float* dev_num_points_per_pillar;
  int* dev_sparse_pillar_map;
  float* dev_pillar_point_feature;
  float* dev_pillar_coors;
  int host_pillar_count[1] = {};
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_points),
                       in_num_points * kNumPointFeature * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_x_coors),
                       kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_y_coors),
                       kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_num_points_per_pillar),
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sparse_pillar_map),
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_point_feature),
                       kMaxNumPillars * kMaxNumPointsPerPillar *
                           kNumPointFeature * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_coors),
                       kMaxNumPillars * 4 * sizeof(float)));

  GPU_CHECK(cudaMemcpy(dev_points, points_array,
                       in_num_points * kNumPointFeature * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemset(dev_x_coors, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(
      cudaMemset(dev_num_points_per_pillar, 0, kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_point_feature, 0,
                       kMaxNumPillars * kMaxNumPointsPerPillar *
                           kNumPointFeature * sizeof(float)));
  GPU_CHECK(
      cudaMemset(dev_pillar_coors, 0, kMaxNumPillars * 4 * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map, 0,
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  test_obj.PreprocessGPU(dev_points, in_num_points, dev_x_coors, dev_y_coors,
                         dev_num_points_per_pillar, dev_pillar_point_feature,
                         dev_pillar_coors, dev_sparse_pillar_map,
                         host_pillar_count);

  int* x_coors = new int[kMaxNumPillars];
  int* y_coors = new int[kMaxNumPillars];
  float* num_points_per_pillar = new float[kMaxNumPillars];
  int* sparse_pillar_map = new int[kNumIndsForScan * kNumIndsForScan];
  float* pillar_point_feature =
      new float[kMaxNumPillars * kMaxNumPointsPerPillar * kNumPointFeature];
  float* pillar_coors = new float[kMaxNumPillars * 4];
  GPU_CHECK(cudaMemcpy(x_coors, dev_x_coors, kMaxNumPillars * sizeof(int),
                       cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(y_coors, dev_y_coors, kMaxNumPillars * sizeof(int),
                       cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(num_points_per_pillar, dev_num_points_per_pillar,
                       kMaxNumPillars * sizeof(float), cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(sparse_pillar_map, dev_sparse_pillar_map,
                       kNumIndsForScan * kNumIndsForScan * sizeof(int),
                       cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(pillar_point_feature, dev_pillar_point_feature,
                       kMaxNumPillars * kMaxNumPointsPerPillar *
                           kNumPointFeature * sizeof(float),
                       cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(pillar_coors, dev_pillar_coors,
                       kMaxNumPillars * 4 * sizeof(float),
                       cudaMemcpyDeviceToHost));

  EXPECT_EQ(x_coors[320], static_cast<int>(pillar_coors[1283]));
  EXPECT_EQ(y_coors[44], static_cast<int>(pillar_coors[178]));
  EXPECT_GT(pillar_coors[10930 * 4 + 2], 0.9);
  EXPECT_GT(pillar_coors[10930 * 4 + 3], 0.9);
  EXPECT_EQ(pillar_coors[10931 * 4 + 2], 0);
  EXPECT_EQ(pillar_coors[10931 * 4 + 3], 0);
  EXPECT_GT(num_points_per_pillar[10930], 0.9);
  EXPECT_EQ(num_points_per_pillar[10931], 0);
  EXPECT_EQ(host_pillar_count[0], 10931);

  GPU_CHECK(cudaFree(dev_points));
  GPU_CHECK(cudaFree(dev_x_coors));
  GPU_CHECK(cudaFree(dev_y_coors));
  GPU_CHECK(cudaFree(dev_num_points_per_pillar));
  GPU_CHECK(cudaFree(dev_sparse_pillar_map));
  GPU_CHECK(cudaFree(dev_pillar_point_feature));
  GPU_CHECK(cudaFree(dev_pillar_coors));

  delete[] points_array;
  delete[] x_coors;
  delete[] y_coors;
  delete[] num_points_per_pillar;
  delete[] sparse_pillar_map;
  delete[] pillar_point_feature;
  delete[] pillar_coors;
}
*/
/*
// TODO(chenjiahao): should be changed to multi-anchor for multi-class
TEST(TestSuite, CheckGenerateAnchors) {
  const int kNumClass = 1;
  const int kMaxNumPillars = 12000;
  const int kMaxNumPointsPerPillar = 100;
  const int kNumPointFeature = 4;
  const int kGridXSize = 432;
  const int kGridYSize = 496;
  const int kGridZSize = 1;
  const float kPillarXSize = 0.16;
  const float kPillarYSize = 0.16;
  const float kPillarZSize = 4.0;
  const float kMinXRange = 0;
  const float kMinYRange = -39.68;
  const float kMinZRange = -3.0;
  const int kNumIndsForScan = 512;
  const int kNumThreads = 64;
  TestClass test_obj(kNumClass, kMaxNumPillars, kMaxNumPointsPerPillar,
                     kNumPointFeature, kGridXSize, kGridYSize, kGridZSize,
                     kPillarXSize, kPillarYSize, kPillarZSize, kMinXRange,
                     kMinYRange, kMinZRange, kNumIndsForScan, kNumThreads);

  const int kNumAnchor = 432 * 0.5 * 496 * 0.5 * 2;
  float* anchors_px = new float[kNumAnchor];
  float* anchors_py = new float[kNumAnchor];
  float* anchors_pz = new float[kNumAnchor];
  float* anchors_dx = new float[kNumAnchor];
  float* anchors_dy = new float[kNumAnchor];
  float* anchors_dz = new float[kNumAnchor];
  float* anchors_ro = new float[kNumAnchor];
  test_obj.GenerateAnchors(anchors_px, anchors_py, anchors_pz, anchors_dx,
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
  const int kNumClass = 1;
  const int kMaxNumPillars = 12000;
  const int kMaxNumPointsPerPillar = 100;
  const int kNumPointFeature = 4;
  const int kGridXSize = 432;
  const int kGridYSize = 496;
  const int kGridZSize = 1;
  const float kPillarXSize = 0.16;
  const float kPillarYSize = 0.16;
  const float kPillarZSize = 4.0;
  const float kMinXRange = 0;
  const float kMinYRange = -39.68;
  const float kMinZRange = -3.0;
  const int kNumIndsForScan = 512;
  const int kNumThreads = 64;
  TestClass test_obj(kNumClass, kMaxNumPillars, kMaxNumPointsPerPillar,
                     kNumPointFeature, kGridXSize, kGridYSize, kGridZSize,
                     kPillarXSize, kPillarYSize, kPillarZSize, kMinXRange,
                     kMinYRange, kMinZRange, kNumIndsForScan, kNumThreads);

  const int kNumAnchor = 432 * 0.5 * 496 * 0.5 * 2;

  float* anchors_px = new float[kNumAnchor];
  float* anchors_py = new float[kNumAnchor];
  float* anchors_pz = new float[kNumAnchor];
  float* anchors_dx = new float[kNumAnchor];
  float* anchors_dy = new float[kNumAnchor];
  float* anchors_dz = new float[kNumAnchor];
  float* anchors_ro = new float[kNumAnchor];
  float* box_anchors_min_x = new float[kNumAnchor];
  float* box_anchors_min_y = new float[kNumAnchor];
  float* box_anchors_max_x = new float[kNumAnchor];
  float* box_anchors_max_y = new float[kNumAnchor];
  test_obj.GenerateAnchors(anchors_px, anchors_py, anchors_pz, anchors_dx,
                           anchors_dy, anchors_dz, anchors_ro);
  test_obj.ConvertAnchors2BoxAnchors(anchors_px, anchors_py, box_anchors_min_x,
                                     box_anchors_min_y, box_anchors_max_x,
                                     box_anchors_max_y);

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
}*/

TEST(TestSuite, CheckDoInference) {
  const int kNumPointFeature = 5;
  const int kOutputNumBoxFeature = 7;
  const float kNormalizingFactor = 255.0;
  TestClass test_obj;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  apollo::perception::benchmark::PointCloudPtr org_cloud_ptr(
      new pcl::PointCloud<apollo::perception::benchmark::PointXYZIL>);
  std::string file_name =
      "/apollo/modules/perception/testdata/lidar/app/data/0001_00.pcd";

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
  float* points_array = new float[pcl_pc_ptr->size() * kNumPointFeature];
  test_obj.PclXYZITToArray(pcl_pc_ptr, points_array, kNormalizingFactor);

  std::vector<float> out_detections;
  std::vector<int> out_labels;
  test_obj.DoInference(points_array, pcl_pc_ptr->size(), &out_detections,
                       &out_labels);

  int num_objects = out_detections.size() / kOutputNumBoxFeature;
  EXPECT_GE(num_objects, 10);
  EXPECT_EQ(num_objects, out_labels.size());

  for (int j = 0; j < num_objects; ++j) {
    float x = out_detections.at(j * kOutputNumBoxFeature + 0);
    float y = out_detections.at(j * kOutputNumBoxFeature + 1);
    float z = out_detections.at(j * kOutputNumBoxFeature + 2);
    float dx = out_detections.at(j * kOutputNumBoxFeature + 4);
    float dy = out_detections.at(j * kOutputNumBoxFeature + 3);
    float dz = out_detections.at(j * kOutputNumBoxFeature + 5);
    float yaw = out_detections.at(j * kOutputNumBoxFeature + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    yaw = -yaw;

    int label = out_labels.at(j);
    std::cout << "object id: " << j << ", x: " << x << ", y: " << y
              << ", z: " << z << ", dx: " << dx << ", dy: " << dy
              << ", dz: " << dz << ", yaw: " << yaw << ", label: " << label
              << std::endl;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
