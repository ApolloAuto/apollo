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
  void InitLaneObj(Lane* lane);
  void InitJunctionObj(Junction* junction);
  void InitSignalObj(Signal* signal);
  void InitCrosswalkObj(Crosswalk* crosswalk);
  void InitStopSignObj(StopSign* stop_sign);
  void InitYieldSignObj(YieldSign* yield_sign);
  void InitClearAreaObj(ClearArea* clear_area);
  void InitSpeedBumpObj(SpeedBump* speed_bump);
  void InitRoadObj(Road* road);
};

void HDMapCommonTestSuite::InitLaneObj(Lane* lane) {
  lane->mutable_id()->set_id("lane_1");
  CurveSegment* curve_segment = lane->mutable_central_curve()->add_segment();
  LineSegment* line_segment = curve_segment->mutable_line_segment();
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
  LaneSampleAssociation* lane_sample = lane->add_left_sample();
  lane_sample->set_s(0.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(1.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(2.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(3.0);
  lane_sample->set_width(1.2);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(4.0);
  lane_sample->set_width(1.2);

  lane_sample = lane->add_right_sample();
  lane_sample->set_s(0.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(1.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(2.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(3.0);
  lane_sample->set_width(1.2);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(4.0);
  lane_sample->set_width(1.2);

  lane->set_type(Lane::CITY_DRIVING);
}

void HDMapCommonTestSuite::InitJunctionObj(Junction* junction) {
  junction->mutable_id()->set_id("junction_1");
  Polygon* polygon = junction->mutable_polygon();
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

void HDMapCommonTestSuite::InitSignalObj(Signal* signal) {
  signal->mutable_id()->set_id("signal_1");
  Polygon* polygon = signal->mutable_boundary();
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

  Subsignal* sub_signal = signal->add_subsignal();
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

  CurveSegment* curve_segment = signal->add_stop_line()->add_segment();
  LineSegment* line_segment = curve_segment->mutable_line_segment();
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
void HDMapCommonTestSuite::InitCrosswalkObj(Crosswalk* crosswalk) {
  crosswalk->mutable_id()->set_id("crosswalk_1");
  Polygon* polygon = crosswalk->mutable_polygon();
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
void HDMapCommonTestSuite::InitStopSignObj(StopSign* stop_sign) {
  stop_sign->mutable_id()->set_id("stop_sign_1");
  CurveSegment* curve_segment = stop_sign->add_stop_line()->add_segment();
  LineSegment* line_segment = curve_segment->mutable_line_segment();
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
void HDMapCommonTestSuite::InitYieldSignObj(YieldSign* yield_sign) {
  yield_sign->mutable_id()->set_id("yield_sign_1");
  CurveSegment* curve_segment = yield_sign->add_stop_line()->add_segment();
  LineSegment* line_segment = curve_segment->mutable_line_segment();
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
void HDMapCommonTestSuite::InitClearAreaObj(ClearArea* clear_area) {
  clear_area->mutable_id()->set_id("clear_area_1");
  Polygon* polygon = clear_area->mutable_polygon();
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
void HDMapCommonTestSuite::InitSpeedBumpObj(SpeedBump* speed_bump) {
  speed_bump->mutable_id()->set_id("speed_bump_1");
  CurveSegment* curve_segment = speed_bump->add_position()->add_segment();
  LineSegment* line_segment = curve_segment->mutable_line_segment();
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

void HDMapCommonTestSuite::InitRoadObj(Road* road) {
  road->mutable_id()->set_id("road_1");
  road->mutable_junction_id()->set_id("junction_1");

  RoadSection* section = road->add_section();
  section->mutable_id()->set_id("section_1");
  section->add_lane_id()->set_id("section_1_1");
  section->add_lane_id()->set_id("section_1_2");

  section = road->add_section();
  section->mutable_id()->set_id("section_2");
  section->add_lane_id()->set_id("section_2_1");
}

TEST_F(HDMapCommonTestSuite, TestLaneInfo) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_EQ(lane.id().id(), lane_info.id().id());
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
  for (const auto& segment : lane_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
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
  lane_info.GetWidth(2.0, &left_width, &right_width);
  EXPECT_NEAR(1.5, left_width, 1E-3);
  EXPECT_NEAR(1.5, right_width, 1E-3);
  lane_info.GetWidth(3.5, &left_width, &right_width);
  EXPECT_NEAR(1.2, left_width, 1E-3);
  EXPECT_NEAR(1.2, right_width, 1E-3);
  EXPECT_NEAR(4.0, lane_info.total_length(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetWidth) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_NEAR(3.0, lane_info.GetWidth(2.0), 1E-3);
  EXPECT_NEAR(2.4, lane_info.GetWidth(3.5), 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetEffectiveWidth) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_NEAR(3.0, lane_info.GetEffectiveWidth(2.0), 1E-3);
  EXPECT_NEAR(2.4, lane_info.GetEffectiveWidth(3.5), 1E-3);
}

TEST_F(HDMapCommonTestSuite, PointIsOnLane) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  EXPECT_TRUE(lane_info.IsOnLane({1.5, 1.5}));
  EXPECT_TRUE(lane_info.IsOnLane({1.5, 0.5}));
  EXPECT_TRUE(!lane_info.IsOnLane({0.5, 1.5}));
  EXPECT_TRUE(!lane_info.IsOnLane({1.5, 3}));
}

TEST_F(HDMapCommonTestSuite, BoxIsOnLane) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  apollo::common::math::Box2d target_in_box(
      apollo::common::math::LineSegment2d({2, 1}, {3, 1}), 0.5);
  EXPECT_TRUE(lane_info.IsOnLane(target_in_box));

  apollo::common::math::Box2d target_out_box(
      apollo::common::math::LineSegment2d({2, 1}, {3, 1}), 4);
  EXPECT_TRUE(!lane_info.IsOnLane(target_out_box));
}

TEST_F(HDMapCommonTestSuite, GetSmoothPoint) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  auto smooth_point = lane_info.GetSmoothPoint(1.5);
  EXPECT_NEAR(smooth_point.x(), 2.5, 1E-3);
  EXPECT_NEAR(smooth_point.y(), 1.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, DistanceTo) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  double distance = lane_info.DistanceTo({2.5, 3.0});
  EXPECT_NEAR(distance, 2.0, 1E-3);

  distance = lane_info.DistanceTo({0.5, 3.0});
  EXPECT_NEAR(distance, 2.0615, 1E-3);
}

TEST_F(HDMapCommonTestSuite, DistanceToWithMoreInfo) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  apollo::common::math::Vec2d foot_point;
  double s_offset = 0.0;
  int s_offset_index = 0;
  double distance = lane_info.DistanceTo(
      {2.5, 3.0}, &foot_point, &s_offset, &s_offset_index);
  EXPECT_NEAR(distance, 2.0, 1E-3);
  EXPECT_NEAR(foot_point.x(), 2.5, 1E-3);
  EXPECT_NEAR(foot_point.y(), 1.0, 1E-3);
  EXPECT_NEAR(s_offset, 1.5, 1E-3);

  distance = lane_info.DistanceTo(
      {0.5, 3.0}, &foot_point, &s_offset, &s_offset_index);
  EXPECT_NEAR(distance, 2.06155, 1E-3);
  EXPECT_NEAR(foot_point.x(), 1.0, 1E-3);
  EXPECT_NEAR(foot_point.y(), 1.0, 1E-3);
  EXPECT_NEAR(s_offset, 0.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetNearestPoint) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  double distance = 0.0;
  auto nearest_point = lane_info.GetNearestPoint({2.4, 3.0}, &distance);
  EXPECT_NEAR(nearest_point.x(), 2.4, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);

  nearest_point = lane_info.GetNearestPoint({0.5, 3.0}, &distance);
  EXPECT_NEAR(nearest_point.x(), 1.0, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);

  nearest_point = lane_info.GetNearestPoint({10.5, 3.0}, &distance);
  EXPECT_NEAR(nearest_point.x(), 5.0, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetProjection) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  double accumulate_s = 0.0;
  double lateral = 0.0;
  bool success = lane_info.GetProjection({2.4, 3.0}, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, 1.4, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);

  success = lane_info.GetProjection({0.5, 3.0}, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, -0.5, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);

  success = lane_info.GetProjection({10.5, 3.0}, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, 9.5, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestJunctionInfo) {
  Junction junction;
  InitJunctionObj(&junction);
  JunctionInfo junction_info(junction);
  EXPECT_EQ(junction.id().id(), junction_info.id().id());
  EXPECT_EQ(7, junction_info.polygon().points().size());
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR((i + 1) * 1.0, junction_info.polygon().points()[i].x(), 1E-3);
  }
  EXPECT_NEAR(5.0, junction_info.polygon().points()[5].x(), 1E-3);
  EXPECT_NEAR(2.0, junction_info.polygon().points()[5].y(), 1E-3);
  EXPECT_NEAR(1.0, junction_info.polygon().points()[6].x(), 1E-3);
  EXPECT_NEAR(2.0, junction_info.polygon().points()[6].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestSignalInfo) {
  Signal signal;
  InitSignalObj(&signal);
  SignalInfo signal_info(signal);
  EXPECT_EQ(signal.id().id(), signal_info.id().id());
  EXPECT_EQ(4, signal_info.signal().boundary().point_size());

  int segment_size = 0;
  for (const auto& stop_line : signal.stop_line()) {
    segment_size += stop_line.segment(0).line_segment().point_size() - 1;
  }
  EXPECT_EQ(segment_size, signal_info.segments().size());
  for (const auto& segment : signal_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestCrosswalkInfo) {
  Crosswalk crosswalk;
  InitCrosswalkObj(&crosswalk);
  CrosswalkInfo crosswalk_info(crosswalk);
  EXPECT_EQ(crosswalk.id().id(), crosswalk_info.id().id());
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

TEST_F(HDMapCommonTestSuite, TestStopSignInfo) {
  StopSign stop_sign;
  InitStopSignObj(&stop_sign);
  StopSignInfo stop_sign_info(stop_sign);
  EXPECT_EQ(stop_sign.id().id(), stop_sign_info.id().id());
  EXPECT_EQ(stop_sign.stop_line(0).segment(0).line_segment().point_size() - 1,
            stop_sign_info.segments().size());
  for (const auto& segment : stop_sign_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestYieldSignInfo) {
  YieldSign yield_sign;
  InitYieldSignObj(&yield_sign);
  YieldSignInfo yield_sign_info(yield_sign);
  EXPECT_EQ(yield_sign.id().id(), yield_sign_info.id().id());
  EXPECT_EQ(yield_sign.stop_line(0).segment(0).line_segment().point_size() - 1,
            yield_sign_info.segments().size());
  for (const auto& segment : yield_sign_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestClearAreaInfo) {
  ClearArea clear_area;
  InitClearAreaObj(&clear_area);
  ClearAreaInfo clear_area_info(clear_area);
  EXPECT_EQ(clear_area.id().id(), clear_area_info.id().id());
  EXPECT_EQ(4, clear_area_info.clear_area().polygon().point_size());
  EXPECT_NEAR(0.0, clear_area_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(0.0, clear_area_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(3.0, clear_area_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(0.0, clear_area_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(3.0, clear_area_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(3.0, clear_area_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(0.0, clear_area_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(3.0, clear_area_info.polygon().points()[3].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestSpeedBumpInfo) {
  SpeedBump speed_bump;
  InitSpeedBumpObj(&speed_bump);
  SpeedBumpInfo speed_bump_info(speed_bump);
  EXPECT_EQ(speed_bump.id().id(), speed_bump_info.id().id());
  EXPECT_EQ(speed_bump.position(0).segment(0).line_segment().point_size() - 1,
            speed_bump_info.segments().size());
  for (const auto& segment : speed_bump_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestRoadInfo) {
  Road road;
  InitRoadObj(&road);
  RoadInfo road_info(road);
  EXPECT_EQ(road.id().id(), road_info.id().id());
  EXPECT_EQ(2, road_info.sections().size());

  const RoadSection& section0 = road_info.sections()[0];
  EXPECT_EQ(section0.id().id(), "section_1");
  EXPECT_EQ(section0.lane_id_size(), 2);
  EXPECT_EQ(section0.lane_id(0).id(), "section_1_1");
  EXPECT_EQ(section0.lane_id(1).id(), "section_1_2");

  const RoadSection& section1 = road_info.sections()[1];
  EXPECT_EQ(section1.id().id(), "section_2");
  EXPECT_EQ(section1.lane_id_size(), 1);
  EXPECT_EQ(section1.lane_id(0).id(), "section_2_1");
}

}  // namespace hdmap
}  // namespace apollo
