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

/**
 * @file path_data.cc
 **/

#include "modules/planning/common/path/path_data.h"

#include <algorithm>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::math::CartesianFrenetConverter;

bool PathData::SetDiscretizedPath(const DiscretizedPath &path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  discretized_path_ = path;
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    AERROR << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

bool PathData::SetFrenetPath(const FrenetFramePath &frenet_path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  frenet_path_ = frenet_path;
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    AERROR << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

bool PathData::Empty() const {
  return discretized_path_.empty() && frenet_path_.empty();
}

std::list<std::pair<DiscretizedPath, FrenetFramePath>>
    &PathData::path_data_history() {
  return path_data_history_;
}

const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}

void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
  Clear();
  reference_line_ = reference_line;
}

bool PathData::GetPathPointWithPathS(
    const double s, common::PathPoint *const path_point) const {
  *path_point = discretized_path_.Evaluate(s);
  return true;
}

bool PathData::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint *const path_point) const {
  DCHECK_NOTNULL(reference_line_);
  DCHECK_NOTNULL(path_point);
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  if (ref_s < 0) {
    AERROR << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }
  if (ref_s > frenet_path_.back().s()) {
    AERROR << "ref_s is larger than the length of frenet_path_ length ["
           << frenet_path_.back().s() << "].";
    return false;
  }

  uint32_t index = 0;
  const double kDistanceEpsilon = 1e-3;
  for (uint32_t i = 0; i + 1 < frenet_path_.size(); ++i) {
    if (fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon) {
      path_point->CopyFrom(discretized_path_.at(i));
      return true;
    }
    if (frenet_path_.at(i).s() < ref_s && ref_s <= frenet_path_.at(i + 1).s()) {
      index = i;
      break;
    }
  }
  double r = (ref_s - frenet_path_.at(index).s()) /
             (frenet_path_.at(index + 1).s() - frenet_path_.at(index).s());

  const double discretized_path_s = discretized_path_.at(index).s() +
                                    r * (discretized_path_.at(index + 1).s() -
                                         discretized_path_.at(index).s());
  path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

  return true;
}

void PathData::Clear() {
  discretized_path_.clear();
  frenet_path_.clear();
  reference_line_ = nullptr;
}

std::string PathData::DebugString() const {
  const auto limit =
      std::min(discretized_path_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));

  return apollo::common::util::StrCat(
      "[\n",
      apollo::common::util::PrintDebugStringIter(
          discretized_path_.begin(), discretized_path_.begin() + limit, ",\n"),
      "]\n");
}

bool PathData::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
  DCHECK_NOTNULL(discretized_path);
  std::vector<common::PathPoint> path_points;
  for (const common::FrenetFramePoint &frenet_point : frenet_path) {
    common::SLPoint sl_point;
    common::math::Vec2d cartesian_point;
    sl_point.set_s(frenet_point.s());
    sl_point.set_l(frenet_point.l());
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      AERROR << "Fail to convert sl point to xy point";
      return false;
    }
    ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s());
    double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    ADEBUG << "frenet_point: " << frenet_point.ShortDebugString();
    double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    common::PathPoint path_point = common::util::MakePathPoint(
        cartesian_point.x(), cartesian_point.y(), 0.0, theta, kappa, 0.0, 0.0);

    if (path_points.empty()) {
      path_point.set_s(0.0);
      path_point.set_dkappa(0.0);
    } else {
      common::math::Vec2d last(path_points.back().x(), path_points.back().y());
      common::math::Vec2d current(path_point.x(), path_point.y());
      double distance = (last - current).Length();
      path_point.set_s(path_points.back().s() + distance);
      path_point.set_dkappa((path_point.kappa() - path_points.back().kappa()) /
                            distance);
    }
    path_points.push_back(std::move(path_point));
  }
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

bool PathData::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
  CHECK_NOTNULL(frenet_path);
  CHECK_NOTNULL(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->Length();
  for (const auto &path_point : discretized_path) {
    common::FrenetFramePoint frenet_point =
        reference_line_->GetFrenetPoint(path_point);
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->XYToSL({path_point.x(), path_point.y()},
                                   &sl_point)) {
        AERROR << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      common::FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

bool PathData::LeftTrimWithRefS(const common::FrenetFramePoint &frenet_point) {
  CHECK_NOTNULL(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  frenet_frame_points.emplace_back(frenet_point);

  for (const common::FrenetFramePoint fp : frenet_path_) {
    if (std::fabs(fp.s() - frenet_point.s()) < 1e-6) {
      continue;
    }
    if (fp.s() > frenet_point.s()) {
      frenet_frame_points.push_back(std::move(fp));
    }
  }
  const FrenetFramePath frenet_path =
      FrenetFramePath(std::move(frenet_frame_points));
  SetFrenetPath(frenet_path);
  return true;
}

bool PathData::UpdateFrenetFramePath(const ReferenceLine *reference_line) {
  reference_line_ = reference_line;
  return SetDiscretizedPath(discretized_path_);
}

}  // namespace planning
}  // namespace apollo
