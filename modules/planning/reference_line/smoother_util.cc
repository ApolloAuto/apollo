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

#include <iostream>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"

DEFINE_string(input_file, "", "input file with format x,y per line");
DEFINE_string(output_file, "", "output file with format x,y per line");
DEFINE_double(anchor_step_length, 5.0, "Smooth every 5 meters");
DEFINE_double(smooth_length, 200, "Smooth this amount of length ");
DEFINE_double(lateral_bound, 0.2, "Lateral bound");
DEFINE_double(longitudinal_bound, 1.0, "Longitudinal bound");

namespace apollo {
namespace planning {

using common::math::LineSegment2d;
using common::math::Vec2d;
using common::util::DistanceXY;
using hdmap::MapPathPoint;

class SmootherUtil {
 public:
  explicit SmootherUtil(const std::string& filename) {
    filename_ = filename;
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    std::string point_str;
    while (std::getline(ifs, point_str)) {
      std::size_t idx = point_str.find(',');
      if (idx == std::string::npos) {
        continue;
      }
      auto x_str = point_str.substr(0, idx);
      auto y_str = point_str.substr(idx + 1);
      raw_points_.emplace_back(std::stod(x_str), std::stod(y_str));
    }
    CHECK(common::util::GetProtoFromFile(FLAGS_smoother_config_filename,
                                         &config_))
        << "Failed to read smoother config file: "
        << FLAGS_smoother_config_filename;
  }

  bool Smooth() {
    if (raw_points_.size() <= 2) {
      AERROR << "the original point size is " << raw_points_.size();
      return false;
    }
    for (std::size_t i = 1; i < raw_points_.size(); ++i) {
      std::vector<ReferencePoint> ref_points;
      std::size_t j = i;
      for (double s = 0.0; s < FLAGS_smooth_length && j < raw_points_.size();
           ++j) {
        LineSegment2d segment(raw_points_[j - 1], raw_points_[j]);
        ref_points.emplace_back(MapPathPoint(raw_points_[j], segment.heading()),
                                0.0, 0.0, 0.0, 0.0);
        s += segment.length();
      }
      i = std::max(i, j - 1);
      raw_reference_lines_.emplace_back(ref_points);
    }

    for (std::size_t i = 0; i < raw_reference_lines_.size(); ++i) {
      const auto& raw_ref_line = raw_reference_lines_[i];
      ReferencePoint init_point;
      if (i == 0) {
        init_point = raw_ref_line.GetReferencePoint(0.0);
      } else {
        init_point = raw_reference_lines_[i - 1].reference_points().back();
      }
      const auto anchor_points = CreateAnchorPoints(init_point, raw_ref_line);

      std::unique_ptr<ReferenceLineSmoother> smoother_ptr(
          new QpSplineReferenceLineSmoother(config_));
      smoother_ptr->SetAnchorPoints(anchor_points);
      ReferenceLine ref_line;
      if (!smoother_ptr->Smooth(raw_reference_lines_[i], &ref_line)) {
        AERROR << "smooth failed with reference line points: "
               << raw_reference_lines_[i].reference_points().size()
               << " Length: " << raw_reference_lines_[i].Length();
        return false;
      }
      reference_points_.insert(reference_points_.end(),
                               ref_line.reference_points().begin() + 1,
                               ref_line.reference_points().end());
    }
    return true;
  }

  void Export(const std::string& filename) {
    std::ofstream ofs(filename.c_str());
    ofs.precision(6);
    double s = 0.0;
    for (std::size_t i = 0; i + 1 < reference_points_.size(); ++i) {
      const auto& point = reference_points_[i];
      ofs << std::fixed << "{\"kappa\": " << point.kappa() << ", \"s\": " << s
          << ", \"theta\": " << point.heading() << ", \"x\":" << point.x()
          << ", \"y\":" << point.y() << ", \"dkappa\":" << point.dkappa() << "}"
          << std::endl;
      s += DistanceXY(reference_points_[i + 1], reference_points_[i]);
    }
    ofs.close();
    AINFO << "Smoothed result saved to " << filename;
  }

 private:
  std::vector<AnchorPoint> CreateAnchorPoints(const ReferencePoint& init_point,
                                              const ReferenceLine& ref_line) {
    std::vector<AnchorPoint> anchor_points;
    int num_of_anchors = std::max(
        2,
        static_cast<int>(ref_line.Length() / FLAGS_anchor_step_length + 0.5));
    std::vector<double> anchor_s;
    common::util::uniform_slice(0.0, ref_line.Length(), num_of_anchors - 1,
                                &anchor_s);
    bool set_init_point = false;
    for (const double s : anchor_s) {
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
      anchor.lateral_bound = FLAGS_lateral_bound;
      anchor.longitudinal_bound = FLAGS_longitudinal_bound;
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
  std::vector<ReferenceLine> raw_reference_lines_;
  std::vector<ReferencePoint> reference_points_;
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
