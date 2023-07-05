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

/**
 * @file
 **/

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gflags/gflags.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

DEFINE_string(input_file, "", "input file with format x,y per line");
DEFINE_string(output_file, "", "output file with format x,y per line");
DEFINE_double(smooth_length, 200.0, "Smooth this amount of length ");
DEFINE_double(minimum_point_spacing, 5.0,
              "The minimum distance for input points.");

namespace apollo {
namespace planning {

class SpiralSmootherUtil {
 public:
  static std::vector<Eigen::Vector2d> ReadCoordinatesFromFile(
      const std::string& filename) {
    std::vector<Eigen::Vector2d> raw_points;
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    std::string point_str;

    auto spacing_thres =
        FLAGS_minimum_point_spacing * FLAGS_minimum_point_spacing;

    while (std::getline(ifs, point_str)) {
      size_t idx = point_str.find(',');
      if (idx == std::string::npos) {
        continue;
      }
      auto x_str = point_str.substr(0, idx);
      auto y_str = point_str.substr(idx + 1);

      auto x = std::stod(x_str);
      auto y = std::stod(y_str);

      if (raw_points.size() > 0) {
        auto last_x = raw_points.back().x();
        auto last_y = raw_points.back().y();

        auto dx = x - last_x;
        auto dy = y - last_y;
        if (dx * dx + dy * dy < spacing_thres) {
          continue;
        }
      }
      raw_points.emplace_back(x, y);
    }
    return raw_points;
  }

  static bool Smooth(std::vector<Eigen::Vector2d> raw_points,
                     std::vector<common::PathPoint>* ptr_smooth_points) {
    if (raw_points.size() <= 2) {
      AERROR << "the original point size is " << raw_points.size();
      return false;
    }

    std::vector<Eigen::Vector2d> processed_points;
    processed_points.push_back(raw_points.front());

    for (const Eigen::Vector2d& p : raw_points) {
      Eigen::Vector2d d = p - processed_points.back();
      if (d.norm() < FLAGS_minimum_point_spacing) {
        continue;
      }
      processed_points.push_back(p);
    }

    if (processed_points.size() < 2) {
      processed_points.push_back(raw_points.back());
    }

    Eigen::Vector2d start_point = processed_points.front();
    std::for_each(processed_points.begin(), processed_points.end(),
                  [&start_point](Eigen::Vector2d& p) { p = p - start_point; });

    ReferenceLineSmootherConfig config;
    ACHECK(cyber::common::GetProtoFromFile(
        "modules/planning/conf/spiral_smoother_config.pb.txt", &config));

    std::vector<double> opt_theta;
    std::vector<double> opt_kappa;
    std::vector<double> opt_dkappa;
    std::vector<double> opt_s;
    std::vector<double> opt_x;
    std::vector<double> opt_y;

    SpiralReferenceLineSmoother spiral_smoother(config);
    auto res = spiral_smoother.SmoothStandAlone(processed_points, &opt_theta,
                                                &opt_kappa, &opt_dkappa, &opt_s,
                                                &opt_x, &opt_y);

    if (!res) {
      AWARN << "Optimization failed; the result may not be smooth";
    } else {
      AINFO << "Optimal solution found";
    }

    std::for_each(opt_x.begin(), opt_x.end(),
                  [&start_point](double& x) { x += start_point.x(); });
    std::for_each(opt_y.begin(), opt_y.end(),
                  [&start_point](double& y) { y += start_point.y(); });

    *ptr_smooth_points =
        spiral_smoother.Interpolate(opt_theta, opt_kappa, opt_dkappa, opt_s,
                                    opt_x, opt_y, config.resolution());

    return true;
  }

  static void Export(const std::string& filename,
                     const std::vector<common::PathPoint>& smoothed_points) {
    std::ofstream ofs(filename.c_str());
    if (ofs.fail()) {
      AERROR << "Fail to open file " << filename;
      return;
    }
    ofs.precision(12);
    // skip the first point and the last point
    for (size_t i = 1; i + 1 < smoothed_points.size(); ++i) {
      const auto& point = smoothed_points[i];
      ofs << std::fixed << "{\"kappa\": " << point.kappa()
          << ", \"s\": " << point.s() << ", \"theta\": " << point.theta()
          << ", \"x\":" << point.x() << ", \"y\":" << point.y()
          << ", \"dkappa\":" << point.dkappa() << "}";
    }
    ofs.close();
    AINFO << "Smoothed result saved to " << filename;
  }
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

  std::vector<Eigen::Vector2d> raw_points =
      apollo::planning::SpiralSmootherUtil::ReadCoordinatesFromFile(
          FLAGS_input_file);

  std::vector<apollo::common::PathPoint> smooth_points;
  auto res =
      apollo::planning::SpiralSmootherUtil::Smooth(raw_points, &smooth_points);
  if (!res) {
    AERROR << "Failed to smooth a the line";
  }

  if (FLAGS_output_file.empty()) {
    FLAGS_output_file = FLAGS_input_file + ".smoothed";
    AINFO << "Output file not provided, set to: " << FLAGS_output_file;
  }
  apollo::planning::SpiralSmootherUtil::Export(FLAGS_output_file,
                                               smooth_points);
  return 0;
}
