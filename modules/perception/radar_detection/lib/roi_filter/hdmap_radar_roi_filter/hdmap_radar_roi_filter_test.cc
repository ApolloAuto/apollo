/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar_detection/lib/roi_filter/hdmap_radar_roi_filter/hdmap_radar_roi_filter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace radar {

TEST(HdmapRadarRoiFilterTest, roi_filter) {
  HdmapRadarRoiFilter roi_filter;
  RoiFilterInitOptions init_options;
  bool init_result = roi_filter.Init(init_options);
  EXPECT_TRUE(init_result);
  RoiFilterOptions options;
  base::FramePtr radar_frame(new base::Frame);
  base::ObjectPtr obj(new base::Object);
  obj->center = Eigen::Vector3d(0, 0, 0);
  radar_frame->objects.push_back(obj);
  roi_filter.RoiFilter(options, radar_frame);
  EXPECT_EQ(radar_frame->objects.size(), 1);

  options.roi.reset(new base::HdmapStruct());
  radar_frame->objects.clear();
  radar_frame->objects.push_back(obj);
  options.roi->road_polygons.resize(1);
  base::PointD pt;
  pt.x = -1.0;
  pt.y = -1.0;
  pt.z = 0;
  options.roi->road_polygons[0].push_back(pt);
  pt.x = 1.0;
  pt.y = -1.0;
  pt.z = 0;
  options.roi->road_polygons[0].push_back(pt);
  pt.x = 0;
  pt.y = 1;
  pt.z = 0;
  options.roi->road_polygons[0].push_back(pt);
  roi_filter.RoiFilter(options, radar_frame);
  EXPECT_EQ(radar_frame->objects.size(), 1);

  options.roi->road_polygons[0][2].y = -0.1;
  radar_frame->objects.clear();
  radar_frame->objects.push_back(obj);
  roi_filter.RoiFilter(options, radar_frame);
  EXPECT_EQ(radar_frame->objects.size(), 0);
}

TEST(HdmapRadarRoiFilter, name) {
  HdmapRadarRoiFilter roi_filter;
  EXPECT_EQ(roi_filter.Name(), "HdmapRadarRoiFilter");
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
