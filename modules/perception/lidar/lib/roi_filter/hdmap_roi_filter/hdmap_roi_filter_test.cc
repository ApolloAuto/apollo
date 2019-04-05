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

#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"

#include "gtest/gtest.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/polygon_scan_cvter.h"

namespace apollo {
namespace perception {
namespace lidar {

typedef Bitmap2D::DirectionMajor DirectionMajor;
typedef Bitmap2D::Vec2ui Vec2ui;

typedef PolygonScanCvter<double>::DirectionMajor PolyDirMajor;
typedef PolygonScanCvter<double>::Polygon Polygon;
typedef PolygonScanCvter<double>::Edge Edge;
typedef PolygonScanCvter<double>::Point Point;
typedef std::pair<double, double> IntervalOut;
typedef std::pair<double, double> IntervalIn;

//  bitmap2d test
TEST(hdmap_roi_filter_bitmap2d_test, test_bitmap) {
  Bitmap2D bitmap;
  Eigen::Vector2d min_range(0.0, 0.0);
  Eigen::Vector2d max_range(70.0, 70.0);
  Eigen::Vector2d cell_size(1.0, 1.0);
  DirectionMajor major_dir = DirectionMajor::XMAJOR;
  bitmap.Init(min_range, max_range, cell_size);
  EXPECT_EQ(bitmap.min_range()[0], min_range[0]);
  EXPECT_EQ(bitmap.max_range()[0], max_range[0]);
  EXPECT_EQ(bitmap.cell_size()[0], cell_size[0]);
  EXPECT_EQ(bitmap.dir_major(), 0);
  EXPECT_EQ(bitmap.op_dir_major(), 1);
  EXPECT_EQ(bitmap.dims()[0], 71);
  EXPECT_EQ(bitmap.dims()[1], 71);
  bitmap.SetUp(major_dir);
  EXPECT_EQ(bitmap.map_size()[0], 71);
  EXPECT_EQ(bitmap.map_size()[1], 2);

  EXPECT_EQ(bitmap.bitmap().size(), 71 * 2);
  EXPECT_EQ(bitmap.bitmap()[0], 0);
  // same bit
  bitmap.Set(0.2, 0.2, 1.2);
  EXPECT_EQ(bitmap.bitmap()[0], 3);
  bitmap.Reset(0.2, 0.2, 1.2);
  EXPECT_EQ(bitmap.bitmap()[0], 0);

  // two bits
  bitmap.Set(0.2, 63.2, 66.2);
  EXPECT_EQ(bitmap.bitmap()[0], (1ll << 63));
  EXPECT_EQ(bitmap.bitmap()[1], 3);
  bitmap.Reset(0.2, 63.2, 66.2);
  EXPECT_EQ(bitmap.bitmap()[0], 0);
  EXPECT_EQ(bitmap.bitmap()[1], 0);

  bitmap.Set(Eigen::Vector2d(0.2, 1.2));
  EXPECT_TRUE(bitmap.Check(Eigen::Vector2d(0.2, 1.2)));
  bitmap.Reset(Eigen::Vector2d(0.2, 1.2));
  EXPECT_FALSE(bitmap.Check(Eigen::Vector2d(0.2, 1.2)));

  EXPECT_FALSE(bitmap.IsExists(Eigen::Vector2d(-1.0, -1.0)));
  EXPECT_FALSE(bitmap.IsExists(Eigen::Vector2d(1.0, -1.0)));
  EXPECT_FALSE(bitmap.IsExists(Eigen::Vector2d(-1.0, 1.0)));
  EXPECT_FALSE(bitmap.IsExists(Eigen::Vector2d(80.0, 80.0)));
  EXPECT_TRUE(bitmap.IsExists(Eigen::Vector2d(1.0, 1.0)));

  AINFO << bitmap;
}

// polygon scan test
TEST(hdmap_roi_filter_bitmap2d_test, test_polygon_scan_cvter) {
  Edge edge;
  EXPECT_EQ(edge.min_x, 0.0);
  AINFO << edge;

  Polygon polygon;
  polygon.push_back(Point(1.0, 3.0));
  polygon.push_back(Point(2.0, 8.0));
  polygon.push_back(Point(7.0, 6.0));
  polygon.push_back(Point(9.0, 10.0));
  polygon.push_back(Point(13.0, 5.0));
  polygon.push_back(Point(5.0, 1.0));

  PolygonScanCvter<double> poly_scan_cvter;
  poly_scan_cvter.Init(polygon);
  EXPECT_EQ(poly_scan_cvter.polygon().size(), polygon.size());

  std::vector<std::vector<IntervalOut>> scans_intervals;
  double beg = 0.0, end = 12.0, step = 1.0;
  size_t scan_size = static_cast<int>((end - beg) / step);
  const size_t result_edge_table[] = {0, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 0};
  // const size_t result_no_edge_table[] = {0, 1, 1, 1, 1, 1, 2, 2, 1, 1, 0, 0};
  scans_intervals.resize(scan_size);
  for (double i = beg; i < end; i += step) {
    size_t index = static_cast<int>((i - beg) / step);
    poly_scan_cvter.ScanCvt(i, PolyDirMajor::YMAJOR, &scans_intervals[index]);
    EXPECT_EQ(scans_intervals[index].size(), result_edge_table[index]);
  }

  IntervalIn valid_range(beg, end);
  scans_intervals.clear();
  /*
   * TODO(perception): add back the test.
  poly_scan_cvter.ScansCvt(valid_range, PolyDirMajor::YMAJOR, step,
                           &(scans_intervals));
  for (size_t i = 0; i < scans_intervals.size(); ++i) {
    EXPECT_EQ(scans_intervals[i].size(), result_no_edge_table[i]);
  }
  */
}

bool LoadFrameData(LidarFrame* frame) {
  std::ifstream fin;
  fin.open(
      "/apollo/modules/perception/testdata/lidar/lib/roi_filter/"
      "hdmap_roi_filter/data/"
      "poly_mask_ut.poly");
  CHECK_EQ(fin.fail(), false);
  size_t polygons_num = 0;
  fin >> polygons_num;
  frame->hdmap_struct->junction_polygons.resize(polygons_num);
  for (size_t i = 0; i < polygons_num; ++i) {
    size_t polygon_size = 0;
    fin >> polygon_size;
    frame->hdmap_struct->junction_polygons[i].resize(polygon_size);
    for (size_t j = 0; j < polygon_size; ++j) {
      auto& pt = frame->hdmap_struct->junction_polygons[i].at(j);
      fin >> pt.x >> pt.y >> pt.z;
    }
  }
  fin.close();

  fin.open(
      "/apollo/modules/perception/testdata/lidar/lib/roi_filter/"
      "hdmap_roi_filter/data/"
      "poly_mask_ut.pcd");
  CHECK_EQ(fin.fail(), false);
  size_t cloud_size = 0;
  fin >> cloud_size;
  frame->cloud->resize(cloud_size);
  for (size_t i = 0; i < cloud_size; ++i) {
    auto& pt = frame->cloud->at(i);
    fin >> pt.x >> pt.y >> pt.z;
  }
  return true;
}

class HdmapROIFilterTest : public ::testing::Test {
 public:
  HdmapROIFilterTest() : hdmap_roi_filter_ptr_(new HdmapROIFilter) {
    // prepare test data
    char cyber_path[50] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[50] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_work_root =
        "/apollo/modules/perception/testdata/"
        "lidar/lib/roi_filter/hdmap_roi_filter";
  }

 protected:
  void Init() {
    hdmap_roi_filter_ptr_->Init(opts_);
    EXPECT_LT(hdmap_roi_filter_ptr_->range_ - 70.0, 10e-6);
    EXPECT_LT(hdmap_roi_filter_ptr_->cell_size_ - 0.25, 10e-6);
    EXPECT_LT(hdmap_roi_filter_ptr_->extend_dist_ - 0.0, 10e-6);
    EXPECT_TRUE(hdmap_roi_filter_ptr_->no_edge_table_);
    EXPECT_FALSE(hdmap_roi_filter_ptr_->set_roi_service_);
  }

  void Filter() {
    hdmap_roi_filter_ptr_->Init(opts_);
    frame_.cloud = base::PointFCloudPool::Instance().Get();
    frame_.hdmap_struct.reset(new base::HdmapStruct);
    frame_.lidar2world_pose = Eigen::Affine3d::Identity();
    LoadFrameData(&frame_);
    hdmap_roi_filter_ptr_->Filter(options_, &frame_);

    // compare with ground truth
    size_t valid_num = 0;
    for (size_t i = 0; i < frame_.cloud->size(); ++i) {
      auto& pt = frame_.cloud->at(i);
      if (pt.z > 0) {
        EXPECT_LE(valid_num, frame_.roi_indices.indices.size());
        EXPECT_EQ(frame_.roi_indices.indices[valid_num++], i);
      }
    }
    EXPECT_EQ(valid_num, frame_.roi_indices.indices.size());
  }

  void SimpleCaseFilter() {
    hdmap_roi_filter_ptr_->Init(opts_);
    // check hdmap_struct nullptr.
    EXPECT_FALSE(hdmap_roi_filter_ptr_->Filter(options_, &frame_));

    // check hdmap_struct nullptr.
    frame_.hdmap_struct.reset(new base::HdmapStruct);
    EXPECT_FALSE(hdmap_roi_filter_ptr_->Filter(options_, &frame_));

    // check empty polygon
    frame_.cloud = base::PointFCloudPool::Instance().Get();
    EXPECT_FALSE(hdmap_roi_filter_ptr_->Filter(options_, &frame_));

    hdmap_roi_filter_ptr_->no_edge_table_ = false;
    frame_.hdmap_struct->road_polygons.resize(1);
    frame_.hdmap_struct->road_polygons[0].resize(6);
    frame_.hdmap_struct->road_polygons[0].at(0).x = 5.0;
    frame_.hdmap_struct->road_polygons[0].at(0).y = 10.0;
    frame_.hdmap_struct->road_polygons[0].at(1).x = 5.0;
    frame_.hdmap_struct->road_polygons[0].at(1).y = 10.0;
    frame_.hdmap_struct->road_polygons[0].at(2).x = 10.0;
    frame_.hdmap_struct->road_polygons[0].at(2).y = 15.0;
    frame_.hdmap_struct->road_polygons[0].at(3).x = 10.0;
    frame_.hdmap_struct->road_polygons[0].at(3).y = 15.0;
    frame_.hdmap_struct->road_polygons[0].at(4).x = 15.0;
    frame_.hdmap_struct->road_polygons[0].at(4).y = 10.0;
    frame_.hdmap_struct->road_polygons[0].at(5).x = 10.0;
    frame_.hdmap_struct->road_polygons[0].at(5).y = 5.0;
    frame_.cloud->resize(2);
    frame_.cloud->at(0).x = 0.0;
    frame_.cloud->at(0).y = 0.0;
    frame_.cloud->at(1).x = 0.0;
    frame_.cloud->at(1).y = -10.0;

    // check not in roi
    {
      frame_.lidar2world_pose = Eigen::Affine3d::Identity();
      EXPECT_FALSE(hdmap_roi_filter_ptr_->Filter(options_, &frame_));
    }

    frame_.lidar2world_pose = Eigen::Translation3d(10.0, 10.0, 0);
    {
      // x dirction
      EXPECT_TRUE(hdmap_roi_filter_ptr_->Filter(options_, &frame_));
      EXPECT_EQ(frame_.roi_indices.indices.size(), 1);
      EXPECT_EQ(frame_.roi_indices.indices[0], 0);
    }

    {
      // y direction
      frame_.hdmap_struct->road_polygons[0].at(1).y = 14.0;
      frame_.hdmap_struct->road_polygons[0].at(3).y = 6.0;
      EXPECT_TRUE(hdmap_roi_filter_ptr_->Filter(options_, &frame_));
      EXPECT_EQ(frame_.roi_indices.indices.size(), 1);
      EXPECT_EQ(frame_.roi_indices.indices[0], 0);
    }
  }

  void FilterWithEdgeTable() {
    hdmap_roi_filter_ptr_->set_roi_service_ = false;
    hdmap_roi_filter_ptr_->no_edge_table_ = false;
    Filter();
  }

  void FilterWithParallel() {
    hdmap_roi_filter_ptr_->set_roi_service_ = true;
    hdmap_roi_filter_ptr_->no_edge_table_ = true;
    Filter();
  }

  // input data
  LidarFrame frame_;
  ROIFilterOptions options_;
  ROIFilterInitOptions opts_;

  // algorithm member
  std::unique_ptr<HdmapROIFilter> hdmap_roi_filter_ptr_;
};

// test init from config manager
TEST_F(HdmapROIFilterTest, init) { HdmapROIFilterTest::Init(); }

// test the filter function
TEST_F(HdmapROIFilterTest, filter_with_edge_table) {
  HdmapROIFilterTest::FilterWithEdgeTable();
}

TEST_F(HdmapROIFilterTest, filter_with_parallel) {
  HdmapROIFilterTest::FilterWithParallel();
}

TEST_F(HdmapROIFilterTest, filter_with_simple_case) {
  // TODO(perception): fix the test.
  // HdmapROIFilterTest::SimpleCaseFilter();
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
