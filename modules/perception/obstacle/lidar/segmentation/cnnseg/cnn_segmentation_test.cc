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

#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.h"

#include <cstdio>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/common/log.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/object.h"

// #define VISUALIZE

using apollo::perception::CNNSegmentation;
using apollo::perception::pcl_util::PointCloud;
using apollo::perception::pcl_util::PointCloudPtr;
using apollo::perception::pcl_util::PointIndices;
using apollo::perception::pcl_util::PointXYZIT;
using std::shared_ptr;
using std::string;
using std::unordered_set;
using std::vector;

namespace apollo {
namespace perception {
namespace test {

DEFINE_string(test_dir, "/apollo/modules/perception/data/cnnseg_test/",
              "test data directory");
DEFINE_string(pcd_name, "uscar_12_1470770225_1470770492_1349",
              "poind cloud data name");

struct CellStat {
  CellStat() : point_num(0), valid_point_num(0) {}
  int point_num;
  int valid_point_num;
};

int F2I(float val, float ori, float scale) {
  return static_cast<int>(std::floor((ori - val) * scale));
}

cv::Vec3b GetTypeColor(ObjectType type) {
  switch (type) {
    case ObjectType::PEDESTRIAN:
      return cv::Vec3b(255, 128, 128);  // pink
    case ObjectType::BICYCLE:
      return cv::Vec3b(0, 0, 255);  // blue
    case ObjectType::VEHICLE:
      return cv::Vec3b(0, 255, 0);  // green
    default:
      return cv::Vec3b(0, 255, 255);  // yellow
  }
}

class CNNSegmentationTest : public testing::Test {
 protected:
  CNNSegmentationTest() {}
  ~CNNSegmentationTest() {}
  void SetUp() {
    google::InitGoogleLogging("CNNSegmentationTest");
    cnn_segmentor_.reset(new CNNSegmentation());
  }
  void TearDown() {}

 protected:
  shared_ptr<CNNSegmentation> cnn_segmentor_;
};

bool IsValidRowCol(int row, int rows, int col, int cols) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

int RowCol2Grid(int row, int col, int cols) { return row * cols + col; }

bool GetPointCloudFromFile(const string &pcd_file, PointCloudPtr cloud) {
  pcl::PointCloud<PointXYZIT> ori_cloud;
  if (pcl::io::loadPCDFile(pcd_file, ori_cloud) < 0) {
    AERROR << "Failed to load pcd file: " << pcd_file;
    return false;
  }

  cloud->points.reserve(ori_cloud.points.size());
  for (size_t i = 0; i < ori_cloud.points.size(); ++i) {
    apollo::perception::pcl_util::Point point;
    point.x = ori_cloud.points[i].x;
    point.y = ori_cloud.points[i].y;
    point.z = ori_cloud.points[i].z;
    point.intensity = ori_cloud.points[i].intensity;
    if (std::isnan(ori_cloud.points[i].x)) {
      continue;
    }
    cloud->push_back(point);
  }

  return true;
}

void DrawDetection(const PointCloudPtr &pc_ptr, const PointIndices &valid_idx,
                   int rows, int cols, float range,
                   const vector<std::shared_ptr<Object>> &objects,
                   const string &result_file) {
  // create a new image for visualization
  cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(0.0));

  // map points into bird-view grids
  float inv_res_x = 0.5 * static_cast<float>(cols) / range;
  float inv_res_y = 0.5 * static_cast<float>(rows) / range;
  int grids = rows * cols;
  vector<CellStat> view(grids);

  const std::vector<int> *valid_indices_in_pc = &(valid_idx.indices);
  CHECK_LE(valid_indices_in_pc->size(), pc_ptr->size());
  unordered_set<int> unique_indices;
  for (size_t i = 0; i < valid_indices_in_pc->size(); ++i) {
    int point_id = valid_indices_in_pc->at(i);
    CHECK(unique_indices.find(point_id) == unique_indices.end());
    unique_indices.insert(point_id);
  }

  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto &point = pc_ptr->points[i];
    // * the coordinates of x and y have been exchanged in feature generation
    // step,
    // so they should be swapped back here.
    int col = F2I(point.y, range, inv_res_x);  // col
    int row = F2I(point.x, range, inv_res_y);  // row
    if (IsValidRowCol(row, rows, col, cols)) {
      // get grid index and count point number for corresponding node
      int grid = RowCol2Grid(row, col, cols);
      view[grid].point_num++;
      if (unique_indices.find(i) != unique_indices.end()) {
        view[grid].valid_point_num++;
      }
    }
  }

  // show grids with grey color
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      int grid = RowCol2Grid(row, col, cols);
      if (view[grid].valid_point_num > 0) {
        img.at<cv::Vec3b>(row, col) = cv::Vec3b(127, 127, 127);
      } else if (view[grid].point_num > 0) {
        img.at<cv::Vec3b>(row, col) = cv::Vec3b(63, 63, 63);
      }
    }
  }

  // show segment grids with tight bounding box
  const cv::Vec3b segm_color(0, 0, 255);  // red

  for (size_t i = 0; i < objects.size(); ++i) {
    const std::shared_ptr<Object> &obj = objects[i];
    CHECK_GT(obj->cloud->size(), 0);

    int x_min = INT_MAX;
    int y_min = INT_MAX;
    int x_max = INT_MIN;
    int y_max = INT_MIN;
    float score = obj->score;
    CHECK_GE(score, 0.0);
    CHECK_LE(score, 1.0);
    for (size_t j = 0; j < obj->cloud->size(); ++j) {
      const auto &point = obj->cloud->points[j];
      int col = F2I(point.y, range, inv_res_x);  // col
      int row = F2I(point.x, range, inv_res_y);  // row
      CHECK(IsValidRowCol(row, rows, col, cols));
      img.at<cv::Vec3b>(row, col) = segm_color * score;
      x_min = std::min(col, x_min);
      y_min = std::min(row, y_min);
      x_max = std::max(col, x_max);
      y_max = std::max(row, y_max);
    }

    // fillConvexPoly(img, list.data(), list.size(), cv::Scalar(positive_prob *
    // segm_color));
    cv::Vec3b bbox_color = GetTypeColor(obj->type);
    rectangle(img, cv::Point(x_min, y_min), cv::Point(x_max, y_max),
              cv::Scalar(bbox_color));
  }

  // write image intensity values into file
  FILE *f_res;
  f_res = fopen(result_file.c_str(), "w");
  fprintf(f_res, "%d %d\n", rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      fprintf(f_res, "%u %u %u\n", img.at<cv::Vec3b>(row, col)[0],
              img.at<cv::Vec3b>(row, col)[1], img.at<cv::Vec3b>(row, col)[2]);
    }
  }
  fclose(f_res);
}

TEST_F(CNNSegmentationTest, test_cnnseg_det) {
  FLAGS_work_root = "modules/perception";
  FLAGS_config_manager_path = "./conf/config_manager.config";

  // generate input point cloud data
  const string in_pcd_file = FLAGS_test_dir + FLAGS_pcd_name + ".pcd";
  AINFO << "pcd file: " << in_pcd_file;
  PointCloudPtr in_pc;
  in_pc.reset(new PointCloud());
  EXPECT_TRUE(GetPointCloudFromFile(in_pcd_file, in_pc));

  PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc->size());
  std::iota(indices.begin(), indices.end(), 0);

  SegmentationOptions options;
  options.origin_cloud = in_pc;

  std::vector<std::shared_ptr<Object>> out_objects;

  // testing initialization function
  EXPECT_TRUE(cnn_segmentor_->Init());

  // testing segment function
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(
        cnn_segmentor_->Segment(in_pc, valid_idx, options, &out_objects));
    EXPECT_EQ(out_objects.size(), 15);
  }

#ifdef VISUALIZE
  // do visualization of segmentation results (output object detections)
  string result_file(FLAGS_test_dir);
  result_file = result_file + FLAGS_pcd_name + "-detection.txt";
  DrawDetection(in_pc, valid_idx, cnn_segmentor_->height(),
                cnn_segmentor_->width(), cnn_segmentor_->range(), out_objects,
                result_file);
#endif
}

}  // namespace test
}  // namespace perception
}  // namespace apollo
