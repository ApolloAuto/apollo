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

#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"

#include "modules/common/log.h"

namespace apollo {
namespace perception {

const char polygon_file_name[] =
    "/apollo/modules/perception/data/hdmap_roi_filter_test/poly_mask_ut.poly";
const char pcd_file_name[] =
    "/apollo/modules/perception/data/hdmap_roi_filter_test/poly_mask_ut.pcd";

bool LoadPolygonFile(const std::string& absolute_file_name,
                     std::vector<PolygonType>* polygons_ptr) {
  std::ifstream polygon_data(absolute_file_name, std::ifstream::in);
  if (!polygon_data) {
    AERROR << "Can not open file: " << absolute_file_name;
    return false;
  }

  size_t polygons_num = 0;
  polygon_data >> polygons_num;

  auto& polygons = *polygons_ptr;
  polygons = std::vector<PolygonType>(polygons_num);

  for (auto& polygon : polygons) {
    size_t points_num = 0;
    polygon_data >> points_num;

    polygon.resize(points_num);
    for (auto& vertex : polygon.points) {
      double x = 0, y = 0, z = 0;
      polygon_data >> x >> y >> z;
      vertex.x = x;
      vertex.y = y;
    }
  }
  return true;
}

class HdmapROIFilterTest : public testing::Test, HdmapROIFilter {
 public:
  HdmapROIFilterTest() : pts_cloud_ptr_(new pcl_util::PointCloud) {}

 protected:
  void SetUp() {
    LoadPolygonFile(polygon_file_name, &polygons_);
    pcl::io::loadPCDFile(pcd_file_name, *pts_cloud_ptr_);
  }

  void TearDown() {}

 protected:
  void init();
  void filter();

  std::unique_ptr<HdmapROIFilter> hdmap_roi_filter_ptr_;
  std::vector<PolygonType> polygons_;
  pcl_util::PointCloudPtr pts_cloud_ptr_;
};

void HdmapROIFilterTest::init() {
  FLAGS_work_root = "/apollo/modules/perception/data";
  FLAGS_config_manager_path = "config_manager_test/config_manager.config";
  ASSERT_TRUE(Init());
}

void HdmapROIFilterTest::filter() {
  pcl_util::PointIndices indices;

  ASSERT_TRUE(FilterWithPolygonMask(pts_cloud_ptr_, polygons_, &indices));

  size_t points_num = pts_cloud_ptr_->size();
  std::vector<bool> is_in_roi(points_num, false);

  for (const auto& id : indices.indices) {
    ASSERT_GE(id, 0);
    ASSERT_LT(id, points_num);
    is_in_roi[id] = true;
  }

  size_t true_positive = 0, false_positive = 0;
  size_t true_negitive = 0, false_negitive = 0;

  for (size_t i = 0; i < points_num; ++i) {
    const auto& pt = pts_cloud_ptr_->points[i];

    if (pt.z > 0) {
      EXPECT_TRUE(is_in_roi[i]);
      if (is_in_roi[i])
        ++true_positive;
      else
        ++false_negitive;
    } else {
      EXPECT_FALSE(is_in_roi[i]);
      if (is_in_roi[i])
        ++false_positive;
      else
        ++true_negitive;
    }
  }

  ADEBUG << "True positive: " << true_positive
         << ", False positive: " << false_positive
         << ", True negitive: " << true_negitive
         << ", False negative: " << false_negitive;
}

TEST_F(HdmapROIFilterTest, test_filter) {
  init();
  filter();
}

}  // namespace perception
}  // namespace apollo
