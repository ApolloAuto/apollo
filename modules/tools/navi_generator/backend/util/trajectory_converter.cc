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
 * @brief This file provides the implementation of the class
 * "TrajectoryConverter".
 */

#include "modules/tools/navi_generator/backend/util/trajectory_converter.h"

#include <proj_api.h>
#include <fstream>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "modules/common/log.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/reference_line_smoother_config.pb.h"

namespace apollo {
namespace navi_generator {
namespace util {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;
using apollo::drivers::gnss::GnssBestPose;
using apollo::localization::LocalizationEstimate;

namespace {
constexpr double kDistanceTolerance = 5.0;
// kSinsRadToDeg = 180 / pi
constexpr double kSinsRadToDeg = 57.295779513;
}  // namespace

bool TrajectoryConverter::ExtractTrajectoryPoints(
    const std::string& bag_filename, double length, bool need_clear_buffer,
    bool is_written_to_file) {
  extracted_filename_ =
      bag_filename.substr(0, bag_filename.size() - 4) + ".txt";

  rosbag::Bag bag;
  try {
    bag.open(bag_filename);  // BagMode is Read by default
  } catch (const rosbag::BagException& e) {
    AERROR << "Can't open the input bag file: " << bag_filename;
    AERROR << "The reason is: " << e.what();
    return false;
  }

  if (need_clear_buffer) {
    raw_points_.clear();
  }

  std::vector<std::string> topics;
  topics.emplace_back("/apollo/localization/pose");
  topics.emplace_back("/apollo/sensor/gnss/best_pose");
  double accumulated_length = 0.0;
  double prev_x = -1.0;
  double prev_y = -1.0;
  for (rosbag::MessageInstance const m :
       rosbag::View(bag, rosbag::TopicQuery(topics))) {
    auto localization_pb = m.instantiate<LocalizationEstimate>();
    auto bestpose_pb = m.instantiate<GnssBestPose>();
    if (localization_pb != nullptr) {
      double x = localization_pb->pose().position().x();
      double y = localization_pb->pose().position().y();
      if (prev_x > 0.0 && prev_y > 0.0) {
        accumulated_length += std::hypot(x - prev_x, y - prev_y);
      }
      if (length > 0.0 && accumulated_length > length) {
        break;
      }
      prev_x = x;
      prev_y = y;
      raw_points_.emplace_back(x, y);
    }
    if (bestpose_pb != nullptr) {
      local_utm_zone_id_ = GetUTMZone(bestpose_pb->longitude());
    }
  }
  bag.close();

  if (is_written_to_file) {
    return SaveRawTrajectoryPoints();
  }

  return true;
}

bool TrajectoryConverter::ExtractTrajectoryPointsFromTwoBags(
    const std::string& first_bag_filename,
    const std::string& second_bag_filename, double length_from_second_bag) {
  if (!ExtractTrajectoryPoints(first_bag_filename)) {
    AERROR << "Can't extract trajectory points from the first bag file: "
           << first_bag_filename;
    return false;
  }
  auto first_bag_size = raw_points_.size();

  if (!second_bag_filename.empty() &&
      !ExtractTrajectoryPoints(second_bag_filename, length_from_second_bag,
                               false, false)) {
    AERROR << "Can't extract trajectory points from the second bag file: "
           << second_bag_filename;
    return false;
  }

  if (raw_points_.size() > first_bag_size) {
    const auto& first_point = raw_points_[first_bag_size - 1];
    const auto& second_point = raw_points_[first_bag_size];
    auto distance_between_stiching_points = std::hypot(
        second_point.x() - first_point.x(), second_point.y() - first_point.y());
    if (distance_between_stiching_points > kDistanceTolerance) {
      AERROR << "The distance between the end point of the first bag and the "
                "starting point of the second bag is too large. They can't be "
                "stitched.";
      return false;
    }
  }

  extracted_filename_ =
      first_bag_filename.substr(0, first_bag_filename.size() - 4) + ".txt";
  return SaveRawTrajectoryPoints();
}

bool TrajectoryConverter::SaveRawTrajectoryPoints() {
  std::ofstream fout(extracted_filename_);
  if (!fout.is_open()) {
    AERROR << "Can't open the output file: " << extracted_filename_;
    return false;
  }
  std::for_each(raw_points_.begin(), raw_points_.end(),
                [&fout](const Vec2d& point) {
                  fout << std::setprecision(32) << point.x() << "," << point.y()
                       << std::endl;
                });

  return true;
}

bool TrajectoryConverter::SaveSmoothedTrajectoryPoints() {
  smoothed_filename_ =
      extracted_filename_.substr(0, extracted_filename_.size() - 4) +
      ".smoothed";
  trajectory_smoother_.Export(smoothed_filename_);
  return true;
}

bool TrajectoryConverter::SmoothTrajectoryPoints() {
  if (extracted_filename_.empty()) {
    AERROR << "Need to provide --input_file";
    return false;
  }
  if (!trajectory_smoother_.Import(extracted_filename_)) {
    return false;
  }
  if (!trajectory_smoother_.Smooth()) {
    return false;
  }

  smoothed_points_ = trajectory_smoother_.smoothed_points();
  return true;
}

bool TrajectoryConverter::GetSmoothedTrajectoryWGS84Points(
    std::vector<apollo::localization::msf::WGS84Corr>* const waypoints) {
  for (auto smoothed_point : smoothed_points_) {
    apollo::localization::msf::WGS84Corr wgs84;
    apollo::localization::msf::UtmXYToLatlon(smoothed_point.x(),
                                             smoothed_point.y(),
                                             local_utm_zone_id_, false, &wgs84);
    wgs84.lat *= kSinsRadToDeg;
    wgs84.log *= kSinsRadToDeg;
    waypoints->emplace_back(wgs84);
  }
  return true;
}

bool TrajectoryConverter::ConvertSmoothedTrajectoryPointsToWGS84(
    const std::vector<planning::ReferencePoint>* const smoothed_points,
    std::vector<apollo::localization::msf::WGS84Corr>* const waypoints) {
  for (auto smoothed_point : *smoothed_points) {
    apollo::localization::msf::WGS84Corr wgs84;
    apollo::localization::msf::UtmXYToLatlon(smoothed_point.x(),
                                             smoothed_point.y(),
                                             local_utm_zone_id_, false, &wgs84);
    wgs84.lat *= kSinsRadToDeg;
    wgs84.log *= kSinsRadToDeg;
    waypoints->emplace_back(wgs84);
  }
  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
