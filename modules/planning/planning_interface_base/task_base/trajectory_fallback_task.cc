/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_interface_base/task_base/trajectory_fallback_task.h"

#include <utility>
#include <vector>

#include "modules/common/util/point_factory.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::util::PointFactory;

constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;

using apollo::common::Status;

apollo::common::Status TrajectoryFallbackTask::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  // Generate fallback path.
  GenerateFallbackPath(frame, reference_line_info);

  if (reference_line_info->speed_data().empty()) {
    AERROR << "Speed fallback due to algorithm failure";
    *reference_line_info->mutable_speed_data() = GenerateFallbackSpeed(
        injector_->ego_info(), FLAGS_speed_fallback_distance);
    AmendSpeedDataForControl(reference_line_info->mutable_speed_data());
  }

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }

  return Status::OK();
}

void TrajectoryFallbackTask::GenerateFallbackPath(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  // path and speed fall back
  if (reference_line_info->path_data().Empty()) {
    AERROR << "Path fallback due to algorithm failure";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
    // Clear speed data when path is re-generated.
    reference_line_info->mutable_speed_data()->clear();
  }

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    if (!RetrieveLastFramePathProfile(
            reference_line_info, frame,
            reference_line_info->mutable_path_data())) {
      const auto& candidate_path_data =
          reference_line_info->GetCandidatePathData();
      for (const auto& path_data : candidate_path_data) {
        if (path_data.path_label().find("self") != std::string::npos) {
          *reference_line_info->mutable_path_data() = path_data;
          AERROR << "Use current frame self lane path as fallback ";
          break;
        }
      }
    }
  }
}

void TrajectoryFallbackTask::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  const double unit_s = 1.0;
  const auto& reference_line = reference_line_info->reference_line();

  auto adc_point = injector_->ego_info()->start_point();
  DCHECK(adc_point.has_path_point());
  const auto adc_point_x = adc_point.path_point().x();
  const auto adc_point_y = adc_point.path_point().y();

  common::SLPoint adc_point_s_l;
  if (!reference_line.XYToSL(adc_point.path_point(), &adc_point_s_l)) {
    AERROR << "Fail to project ADC to reference line when calculating path "
              "fallback. Straight forward path is generated";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double max_s = 100.0;
    for (double s = 0; s < max_s; s += unit_s) {
      path_points.push_back(PointFactory::ToPathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, s, adc_point_heading,
          adc_point_kappa, adc_point_dkappa));
      adc_traversed_x += unit_s * std::cos(adc_point_heading);
      adc_traversed_y += unit_s * std::sin(adc_point_heading);
    }
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }

  // Generate a fallback path along the reference line direction
  const auto adc_s = adc_point_s_l.s();
  const auto& adc_ref_point =
      reference_line.GetReferencePoint(adc_point_x, adc_point_y);
  const double dx = adc_point_x - adc_ref_point.x();
  const double dy = adc_point_y - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  const double max_s = reference_line.Length();
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point = reference_line.GetReferencePoint(s);
    path_points.push_back(PointFactory::ToPathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, s - adc_s,
        ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

bool TrajectoryFallbackTask::RetrieveLastFramePathProfile(
    const ReferenceLineInfo* reference_line_info, const Frame* frame,
    PathData* path_data) {
  const auto* ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == nullptr) {
    AERROR
        << "Last frame doesn't succeed, fail to retrieve last frame path data";
    return false;
  }

  const auto& last_frame_discretized_path =
      ptr_last_frame->current_frame_planned_path();

  path_data->SetDiscretizedPath(last_frame_discretized_path);
  const auto adc_frenet_frame_point_ =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());

  bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point_);
  if (!trim_success) {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point_.ShortDebugString();
    return false;
  }
  AERROR << "Use last frame good path to do speed fallback";
  return true;
}

void TrajectoryFallbackTask::AmendSpeedDataForControl(
    SpeedData* speed_data_ptr) {
  // Iterate the speed data with reverse order to modify the deceleration before
  // stopping. If the value of deceleration is not big enough, control module
  // may not stop the vehicle in time.
  SpeedData& speed_data = *speed_data_ptr;
  const int speed_data_num = speed_data.size();
  int modify_index = speed_data_num - 1;
  for (; modify_index >= 0; --modify_index) {
    auto& data = speed_data[modify_index];
    if (data.v() < FLAGS_near_stop_speed && data.a() <= 0.0 &&
        data.a() > FLAGS_near_stop_deceleration) {
      data.set_a(FLAGS_near_stop_deceleration);
    } else {
      break;
    }
  }
  if (modify_index < 0) {
    modify_index = 0;
  }
  // Update the speed according to the new deceleration.
  double pre_s = speed_data[modify_index].s();
  double pre_v = speed_data[modify_index].v();
  double s = 0.0;
  double v = 0.0;
  const double unit_t = FLAGS_fallback_time_unit;
  for (int i = modify_index + 1; i < speed_data_num; ++i) {
    if (pre_v > 0.0) {
      v = std::fmax(0.0, pre_v + unit_t * FLAGS_near_stop_deceleration);
      s = std::fmax(pre_s, pre_s + 0.5 * (pre_v + (pre_v + v)) * unit_t);
    } else {
      v = 0.0;
      s = pre_s;
    }
    speed_data[i].set_v(v);
    speed_data[i].set_s(s);
    pre_s = s;
    pre_v = v;
  }
}

}  // namespace planning
}  // namespace apollo
