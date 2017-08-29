/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "gtest/gtest.h"
#include "modules/map/hdmap/hdmap_impl.h"

namespace apollo {
namespace hdmap {

class HDMapCommonTestSuite : public ::testing::Test {
 protected:
  HDMapCommonTestSuite() {}
  virtual ~HDMapCommonTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
  void init_lane_obj(apollo::hdmap::Lane* lane);
  void init_junction_obj(apollo::hdmap::Junction* junction);
  void init_signal_obj(apollo::hdmap::Signal* signal);
  void init_crosswalk_obj(apollo::hdmap::Crosswalk* crosswalk);
  void init_stop_sign_obj(apollo::hdmap::StopSign* stop_sign);
  void init_yield_sign_obj(apollo::hdmap::YieldSign* yield_sign);
  void init_road_obj(apollo::hdmap::Road* road);
};

void HDMapCommonTestSuite::init_lane_obj(apollo::hdmap::Lane* lane) {
  lane->mutable_id()->set_id("lane_1");
  apollo::hdmap::CurveSegment* curve_segment =
      lane->mutable_central_curve()->add_segment();
  apollo::hdmap::LineSegment* line_segment =
      curve_segment->mutable_line_segment();
  apollo::common::PointENU* pt = line_segment->add_point();
  pt->set_x(1.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(2.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(3.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(4.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(5.0);
  pt->set_y(1.0);
  apollo::hdmap::LaneSampleAssociation* lane_sample = lane->add_left_sample();
  lane_sample->set_s(0.0);
  lane_sample->set_width(1.0);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(1.0);
  lane_sample->set_width(1.0);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(2.0);
  lane_sample->set_width(1.0);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(3.0);
  lane_sample->set_width(0.8);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(4.0);
  lane_sample->set_width(0.5);

  lane_sample = lane->add_right_sample();
  lane_sample->set_s(0.0);
  lane_sample->set_width(1.0);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(1.0);
  lane_sample->set_width(1.0);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(2.0);
  lane_sample->set_width(1.0);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(3.0);
  lane_sample->set_width(1.0);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(4.0);
  lane_sample->set_width(1.0);
}

void HDMapCommonTestSuite::init_junction_obj(
    apollo::hdmap::Junction* junction) {
  junction->mutable_id()->set_id("junction_1");
  apollo::hdmap::Polygon* polygon = junction->mutable_polygon();
  apollo::common::PointENU* pt = polygon->add_point();
  pt->set_x(1.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(2.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(3.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(4.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(5.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(5.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(5.0);
  pt->set_y(2.0);
  pt = polygon->add_point();
  pt->set_x(1.0);
  pt->set_y(2.0);
  pt = polygon->add_point();
  pt->set_x(1.0);
  pt->set_y(1.0);
}

void HDMapCommonTestSuite::init_signal_obj(apollo::hdmap::Signal* signal) {
  signal->mutable_id()->set_id("signal_1");
  apollo::hdmap::Polygon* polygon = signal->mutable_boundary();
  apollo::common::PointENU* pt = polygon->add_point();
  pt->set_x(1.0);
  pt->set_y(1.0);
  pt->set_z(1.0);
  pt = polygon->add_point();
  pt->set_x(1.0);
  pt->set_y(1.0);
  pt->set_z(5.0);
  pt = polygon->add_point();
  pt->set_x(3.0);
  pt->set_y(1.0);
  pt->set_z(5.0);
  pt = polygon->add_point();
  pt->set_x(3.0);
  pt->set_y(1.0);
  pt->set_z(1.0);

  apollo::hdmap::Subsignal* sub_signal = signal->add_subsignal();
  sub_signal->mutable_id()->set_id("sub_signal_1");
  pt = sub_signal->mutable_location();
  pt->set_x(2.0);
  pt->set_y(1.0);
  pt->set_z(4.0);
  sub_signal = signal->add_subsignal();
  sub_signal->mutable_id()->set_id("sub_signal_2");
  pt = sub_signal->mutable_location();
  pt->set_x(2.0);
  pt->set_y(1.0);
  pt->set_z(3.0);
  sub_signal = signal->add_subsignal();
  sub_signal->mutable_id()->set_id("sub_signal_3");
  pt = sub_signal->mutable_location();
  pt->set_x(2.0);
  pt->set_y(1.0);
  pt->set_z(2.0);

  apollo::hdmap::CurveSegment* curve_segment =
      signal->add_stop_line()->add_segment();
  apollo::hdmap::LineSegment* line_segment =
      curve_segment->mutable_line_segment();
  pt = line_segment->add_point();
  pt->set_x(0.0);
  pt->set_y(4.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(1.0);
  pt->set_y(4.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(2.0);
  pt->set_y(4.0);
  pt->set_z(0.0);

  curve_segment = signal->add_stop_line()->add_segment();
  line_segment = curve_segment->mutable_line_segment();
  pt = line_segment->add_point();
  pt->set_x(2.0);
  pt->set_y(4.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(3.0);
  pt->set_y(4.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(4.0);
  pt->set_y(4.0);
  pt->set_z(0.0);
}
void HDMapCommonTestSuite::init_crosswalk_obj(
    apollo::hdmap::Crosswalk* crosswalk) {
  crosswalk->mutable_id()->set_id("crosswalk_1");
  apollo::hdmap::Polygon* polygon = crosswalk->mutable_polygon();
  apollo::common::PointENU* pt = polygon->add_point();
  pt->set_x(0.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
  pt = polygon->add_point();
  pt->set_x(3.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
  pt = polygon->add_point();
  pt->set_x(3.0);
  pt->set_y(3.0);
  pt->set_z(0.0);
  pt = polygon->add_point();
  pt->set_x(0.0);
  pt->set_y(3.0);
  pt->set_z(0.0);
}
void HDMapCommonTestSuite::init_stop_sign_obj(
    apollo::hdmap::StopSign* stop_sign) {
  stop_sign->mutable_id()->set_id("stop_sign_1");
  apollo::hdmap::CurveSegment* curve_segment =
      stop_sign->add_stop_line()->add_segment();
  apollo::hdmap::LineSegment* line_segment =
      curve_segment->mutable_line_segment();
  apollo::common::PointENU* pt = line_segment->add_point();
  pt->set_x(0.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(1.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(2.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
}
void HDMapCommonTestSuite::init_yield_sign_obj(
    apollo::hdmap::YieldSign* yield_sign) {
  yield_sign->mutable_id()->set_id("yield_sign_1");
  apollo::hdmap::CurveSegment* curve_segment =
      yield_sign->add_stop_line()->add_segment();
  apollo::hdmap::LineSegment* line_segment =
      curve_segment->mutable_line_segment();
  apollo::common::PointENU* pt = line_segment->add_point();
  pt->set_x(0.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(1.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
  pt = line_segment->add_point();
  pt->set_x(2.0);
  pt->set_y(0.0);
  pt->set_z(0.0);
}

void HDMapCommonTestSuite::init_road_obj(apollo::hdmap::Road* road) {
  road->mutable_id()->set_id("road_1");
  road->mutable_junction_id()->set_id("junction_1");

  apollo::hdmap::RoadSection* section = road->add_section();
  section->mutable_id()->set_id("section_1");
  section->add_lane_id()->set_id("section_1_1");
  section->add_lane_id()->set_id("section_1_2");

  section = road->add_section();
  section->mutable_id()->set_id("section_2");
  section->add_lane_id()->set_id("section_2_1");
}

TEST_F(HDMapCommonTestSuite, lane_info) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_STREQ(lane.id().id().c_str(), lane_info.id().id().c_str());
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size(),
            lane_info.points().size());
  for (std::size_t i = 0; i < lane_info.points().size(); ++i) {
    EXPECT_NEAR(lane.central_curve().segment(0).line_segment().point(i).x(),
                lane_info.points()[i].x(), 1E-5);
    EXPECT_NEAR(lane.central_curve().segment(0).line_segment().point(i).y(),
                lane_info.points()[i].y(), 1E-5);
  }
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size() - 1,
            lane_info.segments().size());
  for (std::size_t i = 0; i < lane_info.segments().size(); ++i) {
    EXPECT_NEAR(1.0, lane_info.segments()[i].length(), 1E-4);
  }
  EXPECT_EQ(lane_info.unit_directions().size(),
            lane_info.segments().size() + 1);
  for (std::size_t i = 0; i < lane_info.segments().size(); ++i) {
    EXPECT_EQ(lane_info.segments()[i].unit_direction(),
              lane_info.unit_directions()[i]);
  }
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size(),
            lane_info.accumulate_s().size());
  for (std::size_t i = 0; i < lane_info.accumulate_s().size(); ++i) {
    EXPECT_NEAR(i * 1.0, lane_info.accumulate_s()[i], 1E-4);
  }
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size(),
            lane_info.headings().size());
  for (std::size_t i = 0; i < lane_info.headings().size(); ++i) {
    EXPECT_NEAR(lane_info.unit_directions()[i].Angle(), lane_info.headings()[i],
                1E-3);
  }
  double left_width = 0.0;
  double right_width = 0.0;
  lane_info.get_width(2.0, &left_width, &right_width);
  EXPECT_NEAR(1.0, left_width, 1E-3);
  EXPECT_NEAR(1.0, right_width, 1E-3);
  lane_info.get_width(3.5, &left_width, &right_width);
  EXPECT_NEAR(0.65, left_width, 1E-3);
  EXPECT_NEAR(1.0, right_width, 1E-3);
  EXPECT_NEAR(4.0, lane_info.total_length(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, lane_info_get_width) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_NEAR(2.0, lane_info.get_width(2.0), 1E-3);
  EXPECT_NEAR(1.65, lane_info.get_width(3.5), 1E-3);
}

TEST_F(HDMapCommonTestSuite, lane_info_get_effective_width) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_NEAR(2.0, lane_info.get_effective_width(2.0), 1E-3);
  EXPECT_NEAR(1.3, lane_info.get_effective_width(3.5), 1E-3);
}

TEST_F(HDMapCommonTestSuite, point_is_on_lane) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);

  apollo::common::math::Vec2d target_in_point_1(1.5, 1.5);
  EXPECT_TRUE(lane_info.is_on_lane(target_in_point_1));

  apollo::common::math::Vec2d target_in_point_2(1.5, 0.5);
  EXPECT_TRUE(lane_info.is_on_lane(target_in_point_2));

  apollo::common::math::Vec2d target_in_point_3(0.5, 1.5);
  EXPECT_TRUE(!lane_info.is_on_lane(target_in_point_3));

  apollo::common::math::Vec2d target_out_point_4(1.5, 3);
  EXPECT_TRUE(!lane_info.is_on_lane(target_out_point_4));
}

TEST_F(HDMapCommonTestSuite, box_is_on_lane) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);

  apollo::common::math::Box2d target_in_box(
      apollo::common::math::LineSegment2d({2, 1}, {3, 1}), 0.5);
  EXPECT_TRUE(lane_info.is_on_lane(target_in_box));

  apollo::common::math::Box2d target_out_box(
      apollo::common::math::LineSegment2d({2, 1}, {3, 1}), 4);
  EXPECT_TRUE(!lane_info.is_on_lane(target_out_box));
}

TEST_F(HDMapCommonTestSuite, get_smooth_point) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);

  auto smooth_point = lane_info.get_smooth_point(1.5);
  EXPECT_NEAR(smooth_point.x(), 2.5, 1E-3);
  EXPECT_NEAR(smooth_point.y(), 1.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, distance_to) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);

  apollo::common::math::Vec2d point_1(2.5, 3.0);
  double distance = lane_info.distance_to(point_1);
  EXPECT_NEAR(distance, 2.0, 1E-3);

  apollo::common::math::Vec2d point_2(0.5, 3.0);
  distance = lane_info.distance_to(point_2);
  EXPECT_NEAR(distance, 2.0615, 1E-3);
}

TEST_F(HDMapCommonTestSuite, distance_to_with_more_info) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);

  apollo::common::math::Vec2d foot_point;
  double s_offset = 0.0;
  int s_offset_index = 0;
  apollo::common::math::Vec2d point_1(2.5, 3.0);
  double distance =
      lane_info.distance_to(point_1, &foot_point, &s_offset, &s_offset_index);
  EXPECT_NEAR(distance, 2.0, 1E-3);
  EXPECT_NEAR(foot_point.x(), 2.5, 1E-3);
  EXPECT_NEAR(foot_point.y(), 1.0, 1E-3);
  EXPECT_NEAR(s_offset, 1.5, 1E-3);

  apollo::common::math::Vec2d point_2(0.5, 3.0);
  distance =
      lane_info.distance_to(point_2, &foot_point, &s_offset, &s_offset_index);
  EXPECT_NEAR(distance, 2.06155, 1E-3);
  EXPECT_NEAR(foot_point.x(), 1.0, 1E-3);
  EXPECT_NEAR(foot_point.y(), 1.0, 1E-3);
  EXPECT_NEAR(s_offset, 0.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, get_nearest_point) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);

  apollo::common::math::Vec2d point_1(2.4, 3.0);
  double distance = 0.0;
  auto nearest_point = lane_info.get_nearest_point(point_1, &distance);
  EXPECT_NEAR(nearest_point.x(), 2.4, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);

  apollo::common::math::Vec2d point_2(0.5, 3.0);
  nearest_point = lane_info.get_nearest_point(point_2, &distance);
  EXPECT_NEAR(nearest_point.x(), 1.0, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);

  apollo::common::math::Vec2d point_3(10.5, 3.0);
  nearest_point = lane_info.get_nearest_point(point_3, &distance);
  EXPECT_NEAR(nearest_point.x(), 5.0, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, get_projection) {
  apollo::hdmap::Lane lane;
  init_lane_obj(&lane);
  LaneInfo lane_info(lane);

  double accumulate_s = 0.0;
  double lateral = 0.0;
  apollo::common::math::Vec2d point_1(2.4, 3.0);
  bool success = lane_info.get_projection(point_1, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, 1.4, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);

  apollo::common::math::Vec2d point_2(0.5, 3.0);
  success = lane_info.get_projection(point_2, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, -0.5, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);

  apollo::common::math::Vec2d point_3(10.5, 3.0);
  success = lane_info.get_projection(point_3, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, 9.5, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, junction_info) {
  apollo::hdmap::Junction junction;
  init_junction_obj(&junction);
  JunctionInfo junction_info(junction);
  EXPECT_STREQ(junction.id().id().c_str(), junction_info.id().id().c_str());
  EXPECT_EQ(7, junction_info.polygon().points().size());
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR((i + 1) * 1.0, junction_info.polygon().points()[i].x(), 1E-3);
  }
  EXPECT_NEAR(5.0, junction_info.polygon().points()[5].x(), 1E-3);
  EXPECT_NEAR(2.0, junction_info.polygon().points()[5].y(), 1E-3);
  EXPECT_NEAR(1.0, junction_info.polygon().points()[6].x(), 1E-3);
  EXPECT_NEAR(2.0, junction_info.polygon().points()[6].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, signal_info) {
  apollo::hdmap::Signal signal;
  init_signal_obj(&signal);
  SignalInfo signal_info(signal);
  EXPECT_STREQ(signal.id().id().c_str(), signal_info.id().id().c_str());
  EXPECT_EQ(4, signal_info.signal().boundary().point_size());

  int segment_size = 0;
  for (int i = 0; i < signal.stop_line_size(); ++i) {
    segment_size +=
        signal.stop_line(i).segment(0).line_segment().point_size() - 1;
  }
  EXPECT_EQ(segment_size, signal_info.segments().size());
  for (std::size_t i = 0; i < signal_info.segments().size(); ++i) {
    EXPECT_NEAR(1.0, signal_info.segments()[i].length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, crosswalk_info) {
  apollo::hdmap::Crosswalk crosswalk;
  init_crosswalk_obj(&crosswalk);
  CrosswalkInfo crosswalk_info(crosswalk);
  EXPECT_STREQ(crosswalk.id().id().c_str(), crosswalk_info.id().id().c_str());
  EXPECT_EQ(4, crosswalk_info.crosswalk().polygon().point_size());
  EXPECT_NEAR(0.0, crosswalk_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(0.0, crosswalk_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(3.0, crosswalk_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(0.0, crosswalk_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(3.0, crosswalk_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(3.0, crosswalk_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(0.0, crosswalk_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(3.0, crosswalk_info.polygon().points()[3].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, stop_sign_info) {
  apollo::hdmap::StopSign stop_sign;
  init_stop_sign_obj(&stop_sign);
  StopSignInfo stop_sign_info(stop_sign);
  EXPECT_STREQ(stop_sign.id().id().c_str(), stop_sign_info.id().id().c_str());
  EXPECT_EQ(stop_sign.stop_line(0).segment(0).line_segment().point_size() - 1,
            stop_sign_info.segments().size());
  for (std::size_t i = 0; i < stop_sign_info.segments().size(); ++i) {
    EXPECT_NEAR(1.0, stop_sign_info.segments()[i].length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, yield_sign_info) {
  apollo::hdmap::YieldSign yield_sign;
  init_yield_sign_obj(&yield_sign);
  YieldSignInfo yield_sign_info(yield_sign);
  EXPECT_STREQ(yield_sign.id().id().c_str(), yield_sign_info.id().id().c_str());
  EXPECT_EQ(yield_sign.stop_line(0).segment(0).line_segment().point_size() - 1,
            yield_sign_info.segments().size());
  for (std::size_t i = 0; i < yield_sign_info.segments().size(); ++i) {
    EXPECT_NEAR(1.0, yield_sign_info.segments()[i].length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, road_info) {
  apollo::hdmap::Road road;
  init_road_obj(&road);
  RoadInfo road_info(road);
  EXPECT_EQ(road.id().id(), road_info.id().id());
  EXPECT_EQ(2, road_info.sections().size());

  const apollo::hdmap::RoadSection& section0 = road_info.sections()[0];
  EXPECT_EQ(section0.id().id(), "section_1");
  EXPECT_EQ(section0.lane_id_size(), 2);
  EXPECT_EQ(section0.lane_id(0).id(), "section_1_1");
  EXPECT_EQ(section0.lane_id(1).id(), "section_1_2");

  const apollo::hdmap::RoadSection& section1 = road_info.sections()[1];
  EXPECT_EQ(section1.id().id(), "section_2");
  EXPECT_EQ(section1.lane_id_size(), 1);
  EXPECT_EQ(section1.lane_id(0).id(), "section_2_1");
}

}  // namespace hdmap
}  // namespace apollo
