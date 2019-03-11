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

#include "modules/localization/lmd/predictor/perception/lm_sampler.h"

namespace apollo {
namespace localization {

using apollo::perception::LaneMarker;
using apollo::perception::LaneMarkers;

namespace {
constexpr int kSamplingNum = 10;
}  // namespace

LMSampler::LMSampler() { pc_sourcepoint_.clear(); }

LMSampler::~LMSampler() {}

double LMSampler::get_curve_value(double x_value, double c0, double c1,
                                  double c2, double c3) const {
  return c3 * pow(x_value, 3.0) + c2 * pow(x_value, 2.0) + c1 * x_value + c0;
}

double LMSampler::calculate_derivative(double x_value, double c0, double c1,
                                       double c2, double c3) const {
  return 3 * c3 * pow(x_value, 2.0) + 2 * c2 + c1;
}

double LMSampler::calculate_curvity(double x_value, double c0, double c1,
                                    double c2, double c3) const {
  double derivative = calculate_derivative(x_value, c0, c1, c2, c3);

  return abs(6 * c3 * x_value + 2 * c2) /
         pow(1 + pow(derivative, 2.0), (3.0 / 2));
}

int LMSampler::SamplingForOneLaneMarker(const LaneMarker& lane_marker) {
  PCSourcePoint point;
  for (auto i = 0; i < kSamplingNum; i++) {
    double x_value = 0.15 * i;

    point.position.set_x(x_value);

    point.position.set_y(get_curve_value(
        x_value, lane_marker.c0_position(), lane_marker.c1_heading_angle(),
        lane_marker.c2_curvature(), lane_marker.c3_curvature_derivative()));

    point.position.set_z(0.0);

    point.direction.set_x(1.0);

    point.direction.set_y(calculate_derivative(
        x_value, lane_marker.c0_position(), lane_marker.c1_heading_angle(),
        lane_marker.c2_curvature(), lane_marker.c3_curvature_derivative()));

    point.direction.set_z(0.0);

    point.curvature = calculate_curvity(
        x_value, lane_marker.c0_position(), lane_marker.c1_heading_angle(),
        lane_marker.c2_curvature(), lane_marker.c3_curvature_derivative());

    pc_sourcepoint_.push_back(point);
  }
  return 0;
}

const std::vector<PCSourcePoint>& LMSampler::Sampling(
    const LaneMarkers& lane_markers) {
  pc_sourcepoint_.clear();

  bool b_lanemarker = false;
  if (lane_markers.has_left_lane_marker()) {
    b_lanemarker = true;
    const auto& lanemarker = lane_markers.left_lane_marker();
    SamplingForOneLaneMarker(lanemarker);
  }
  if (lane_markers.has_right_lane_marker()) {
    b_lanemarker = true;
    const auto& lanemarker = lane_markers.right_lane_marker();
    SamplingForOneLaneMarker(lanemarker);
  }

  for (auto i = 0; i < lane_markers.next_left_lane_marker_size(); ++i) {
    b_lanemarker = true;
    const auto& lanemarker = lane_markers.next_left_lane_marker(i);
    SamplingForOneLaneMarker(lanemarker);
  }

  for (auto i = 0; i < lane_markers.next_right_lane_marker_size(); ++i) {
    b_lanemarker = true;
    const auto& lanemarker = lane_markers.next_right_lane_marker(i);
    SamplingForOneLaneMarker(lanemarker);
  }

  if (!b_lanemarker) {
    AERROR << "No any lanemarkers from perception! ";
  }

  return pc_sourcepoint_;
}

}  // namespace localization
}  // namespace apollo
