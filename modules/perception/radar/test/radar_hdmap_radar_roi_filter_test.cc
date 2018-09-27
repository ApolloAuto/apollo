// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: radar_hdmap_radar_roi_filter_test.cc
// @brief: unit test for hdmap_radar_roi_fitler

#include <gtest/gtest.h>
#include "modules/perception/radar/lib/roi_filter/hdmap_radar_roi_filter/hdmap_radar_roi_filter.h"

namespace apollo {
namespace perception {
namespace radar {

TEST(HdmapRadarRoiFilterTest, roi_filter) {
  HdmapRadarRoiFilter roi_filter;
  bool init_result = roi_filter.Init();
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
