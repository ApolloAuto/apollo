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
 * @file
 **/

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/future.h"
#include "modules/common/util/util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_base/reference_line/reference_line_smoother.h"

DEFINE_string(input_file, "", "input file with format x,y per line");
DEFINE_string(output_file, "", "output file with format x,y per line");
DEFINE_double(smooth_length, 200, "Smooth this amount of length ");

namespace apollo {
namespace planning {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;
using apollo::hdmap::MapPathPoint;

class SmootherUtil {
 public:
  explicit SmootherUtil(const std::string& filename) : filename_(filename) {
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    std::string point_str;
    while (std::getline(ifs, point_str)) {
      size_t idx = point_str.find(',');
      if (idx == std::string::npos) {
        continue;
      }
      auto x_str = point_str.substr(0, idx);
      auto y_str = point_str.substr(idx + 1);
      raw_points_.emplace_back(std::stod(x_str), std::stod(y_str));
    }
    ACHECK(cyber::common::GetProtoFromFile(FLAGS_smoother_config_filename,
                                           &config_))
        << "Failed to read smoother config file: "
        << FLAGS_smoother_config_filename;
  }

  bool Smooth() {
    if (raw_points_.size() <= 2) {
      AERROR << "the original point size is " << raw_points_.size();
      return false;
    }
    size_t i = 1;
    {
      std::vector<ReferencePoint> ref_points;
      double s = 0.0;
      for (; s < FLAGS_smooth_length && i < raw_points_.size(); ++i) {
        LineSegment2d segment(raw_points_[i - 1], raw_points_[i]);
        ref_points.emplace_back(MapPathPoint(raw_points_[i], segment.heading()),
                                0.0, 0.0);
        s += segment.length();
      }
      ReferenceLine init_ref(ref_points);
      // Prefer "std::make_unique" to direct use of "new".
      // Reference "https://herbsutter.com/gotw/_102/" for details.
      auto smoother_ptr =
          std::make_unique<QpSplineReferenceLineSmoother>(config_);
      auto anchors =
          CreateAnchorPoints(init_ref.reference_points().front(), init_ref);
      smoother_ptr->SetAnchorPoints(anchors);
      ReferenceLine smoothed_init_ref;
      if (!smoother_ptr->Smooth(init_ref, &smoothed_init_ref)) {
        AERROR << "smooth initial reference line failed";
        return false;
      }
      ref_points_ = smoothed_init_ref.reference_points();
    }
    for (; i < raw_points_.size(); ++i) {
      double s = 0.0;
      size_t j = ref_points_.size() - 1;
      while (j > 0 && s < FLAGS_smooth_length / 2.0) {
        s += DistanceXY(ref_points_[j - 1], ref_points_[j]);
        --j;
      }
      ReferenceLine prev_half_ref(ref_points_.begin() + j, ref_points_.end());
      ref_points_.erase(ref_points_.begin() + j, ref_points_.end());
      common::SLPoint sl;
      prev_half_ref.XYToSL(raw_points_[i], &sl);
      while (sl.s() <= prev_half_ref.Length() && i + 1 < raw_points_.size()) {
        prev_half_ref.XYToSL(raw_points_[i + 1], &sl);
        ++i;
      }
      s = 0.0;
      j = i;
      auto ref_points = prev_half_ref.reference_points();
      while (j + 1 < raw_points_.size() && s < FLAGS_smooth_length / 2.0) {
        Vec2d vec = raw_points_[j + 1] - raw_points_[j];
        s += vec.Length();
        ref_points.emplace_back(MapPathPoint(raw_points_[j], vec.Angle()), 0.0,
                                0.0);
        ++j;
      }
      i = j;
      ReferenceLine local_ref(ref_points);
      auto anchors = CreateAnchorPoints(ref_points.front(), local_ref);
      // Prefer "std::make_unique" to direct use of "new".
      // Reference "https://herbsutter.com/gotw/_102/" for details.
      auto smoother_ptr =
          std::make_unique<QpSplineReferenceLineSmoother>(config_);
      smoother_ptr->SetAnchorPoints(anchors);
      ReferenceLine smoothed_local_ref;
      if (!smoother_ptr->Smooth(local_ref, &smoothed_local_ref)) {
        AERROR << "Failed to smooth reference line";
        return false;
      }
      ref_points_.insert(ref_points_.end(),
                         smoothed_local_ref.reference_points().begin(),
                         smoothed_local_ref.reference_points().end());
    }
    return true;
  }

  void Export(const std::string& filename) {
    std::ofstream ofs(filename.c_str());
    if (ofs.fail()) {
      AERROR << "Fail to open file " << filename;
      return;
    }
    ofs.precision(6);
    double s = 0.0;
    // skip the first point and the last point
    for (size_t i = 1; i + 1 < ref_points_.size(); ++i) {
      const auto& point = ref_points_[i];
      ofs << std::fixed << "{\"kappa\": " << point.kappa() << ", \"s\": " << s
          << ", \"theta\": " << point.heading() << ", \"x\":" << point.x()
          << ", \"y\":" << point.y() << ", \"dkappa\":" << point.dkappa()
          << "}";
      s += DistanceXY(point, ref_points_[i + 1]);
    }
    ofs.close();
    AINFO << "Smoothed result saved to " << filename;
  }

 private:
  std::vector<AnchorPoint> CreateAnchorPoints(const ReferencePoint& init_point,
                                              const ReferenceLine& ref_line) {
    std::vector<AnchorPoint> anchor_points;
    int num_of_anchors = std::max(
        2, static_cast<int>(
               ref_line.Length() / config_.max_constraint_interval() + 0.5));
    std::vector<double> anchor_s;
    common::util::uniform_slice(0.0, ref_line.Length(), num_of_anchors - 1,
                                &anchor_s);
    common::SLPoint sl;
    if (!ref_line.XYToSL(init_point, &sl)) {
      AERROR << "Failed to project init point to reference line";
      return anchor_points;
    }
    bool set_init_point = false;
    for (const double s : anchor_s) {
      if (s + config_.max_constraint_interval() / 2.0 < sl.s()) {
        continue;
      }
      ReferencePoint ref_point;
      if (!set_init_point) {
        set_init_point = true;
        ref_point = init_point;
      } else {
        ref_point = ref_line.GetReferencePoint(s);
      }
      AnchorPoint anchor;
      anchor.path_point.set_x(ref_point.x());
      anchor.path_point.set_y(ref_point.y());
      anchor.path_point.set_z(0.0);
      anchor.path_point.set_s(s);
      anchor.path_point.set_theta(ref_point.heading());
      anchor.path_point.set_kappa(ref_point.kappa());
      anchor.lateral_bound = config_.max_lateral_boundary_bound();
      anchor.longitudinal_bound = config_.longitudinal_boundary_bound();
      anchor_points.emplace_back(anchor);
    }
    anchor_points.front().longitudinal_bound = 0;
    anchor_points.front().lateral_bound = 0;
    anchor_points.front().enforced = true;
    anchor_points.back().longitudinal_bound = 0;
    anchor_points.back().lateral_bound = 0;
    anchor_points.back().enforced = true;
    return anchor_points;
  }

 private:
  std::string filename_;
  std::vector<common::math::Vec2d> raw_points_;
  std::vector<ReferencePoint> ref_points_;
  ReferenceLine smoothed_ref_;
  ReferenceLineSmootherConfig config_;
};

}  // namespace planning
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_input_file.empty()) {
    AERROR << "need to provide --input_file";
    return 0;
  }
  apollo::planning::SmootherUtil smoother_util(FLAGS_input_file);
  if (!smoother_util.Smooth()) {
    AERROR << "Failed to smooth a the line";
  }
  if (FLAGS_output_file.empty()) {
    FLAGS_output_file = FLAGS_input_file + ".smoothed";
    AINFO << "Output file not provided, set to: " << FLAGS_output_file;
  }
  smoother_util.Export(FLAGS_output_file);
  return 0;
}
