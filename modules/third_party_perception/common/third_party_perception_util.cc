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
 */

#include <cmath>

#include "modules/common/log.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/common/third_party_perception_util.h"

/**
 * @namespace apollo::third_party_perception
 * @brief apollo::third_party_perception
 */

namespace apollo {
namespace third_party_perception {

using apollo::common::PointENU;
using apollo::common::Quaternion;
using apollo::hdmap::HDMapUtil;
using apollo::perception::PerceptionObstacle;
using apollo::perception::Point;

double GetAngleFromQuaternion(const Quaternion quaternion) {
  double theta = std::atan2(2.0 * quaternion.qw() * quaternion.qz() +
                                quaternion.qx() * quaternion.qy(),
                            1.0 - 2.0 * (quaternion.qy() * quaternion.qy() +
                                         quaternion.qz() * quaternion.qz())) +
                 std::acos(-1.0) / 2.0;
  return theta;
}

void FillPerceptionPolygon(PerceptionObstacle* const perception_obstacle,
                           const double mid_x, const double mid_y,
                           const double mid_z, const double length,
                           const double width, const double height,
                           const double heading) {
  // Generate a 2D cube whose vertices are given in counter-clock order when
  // viewed from top
  const int sign_l[4] = {1, 1, -1, -1};
  const int sign_w[4] = {1, -1, -1, 1};
  for (int i = 0; i < 4; ++i) {
    perception_obstacle->add_polygon_point();
    perception_obstacle->mutable_polygon_point(i)->set_x(
        mid_x + sign_l[i] * length * std::cos(heading) / 2.0 +
        sign_w[i] * width * std::sin(heading) / 2.0);
    perception_obstacle->mutable_polygon_point(i)->set_y(
        mid_y + sign_l[i] * length * std::sin(heading) / 2.0 -
        sign_w[i] * width * std::cos(heading) / 2.0);
  }
}

double GetDefaultObjectLength(const int object_type) {
  double default_object_length = 0.0;
  switch (object_type) {
    case 0: {
      default_object_length = FLAGS_default_car_length;
      break;
    }
    case 1: {
      default_object_length = FLAGS_default_truck_length;
      break;
    }
    case 2: {
      default_object_length = FLAGS_default_bike_length;
      break;
    }
    case 3: {
      default_object_length = FLAGS_default_ped_length;
      break;
    }
    case 4: {
      default_object_length = FLAGS_default_unknown_length;
      break;
    }
  }
  return default_object_length;
}

double GetDefaultObjectWidth(const int object_type) {
  double default_object_width = 0.0;
  switch (object_type) {
    case 0: {
      default_object_width = FLAGS_default_car_width;
      break;
    }
    case 1: {
      default_object_width = FLAGS_default_truck_width;
      break;
    }
    case 2: {
      default_object_width = FLAGS_default_bike_width;
      break;
    }
    case 3: {
      default_object_width = FLAGS_default_ped_width;
      break;
    }
    case 4: {
      default_object_width = FLAGS_default_unknown_width;
      break;
    }
  }
  return default_object_width;
}

Point SLtoXY(const Point& point, const double theta) {
  return SLtoXY(point.x(), point.y(), theta);
}

Point SLtoXY(const double x, const double y, const double theta) {
  Point converted_point;
  converted_point.set_x(x * std::cos(theta) + y * std::sin(theta));
  converted_point.set_y(x * std::sin(theta) - y * std::cos(theta));
  return converted_point;
}

double Distance(const Point& point1, const Point& point2) {
  double distance =
      std::sqrt((point1.x() - point2.x()) * (point1.x() - point2.x()) +
                (point1.y() - point2.y()) * (point1.y() - point2.y()));
  return distance;
}

double GetNearestLaneHeading(const PointENU& point_enu) {
  auto* hdmap = HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    AERROR << "Failed to get nearest lane for point "
           << point_enu.DebugString();
    return -1.0;
  }

  hdmap::LaneInfoConstPtr nearest_lane;

  double nearest_s;
  double nearest_l;

  int status =
      hdmap->GetNearestLane(point_enu, &nearest_lane, &nearest_s, &nearest_l);
  // TODO(lizh): make it a formal status below
  if (status != 0) {
    AERROR << "Failed to get nearest lane for point "
           << point_enu.DebugString();
    return -1.0;
  }
  double lane_heading = nearest_lane->Heading(nearest_s);
  return lane_heading;
}

double GetNearestLaneHeading(const Point& point) {
  auto* hdmap = HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    AERROR << "Failed to get nearest lane for point " << point.DebugString();
    return -1.0;
  }

  PointENU point_enu;
  point_enu.set_x(point.x());
  point_enu.set_y(point.y());
  point_enu.set_z(point.z());

  return GetNearestLaneHeading(point_enu);
}

double GetNearestLaneHeading(const double x, const double y, const double z) {
  auto* hdmap = HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    AERROR << "Failed to get nearest lane for point (" << x << ", " << y << ", "
           << z << ")";
    return -1.0;
  }

  PointENU point_enu;
  point_enu.set_x(x);
  point_enu.set_y(y);
  point_enu.set_z(z);

  return GetNearestLaneHeading(point_enu);
}

double GetLateralDistanceToNearestLane(const Point& point) {
  auto* hdmap = HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    AERROR << "Failed to get nearest lane for point " << point.DebugString();
    return -1.0;
  }

  PointENU point_enu;
  point_enu.set_x(point.x());
  point_enu.set_y(point.y());
  point_enu.set_z(point.z());
  hdmap::LaneInfoConstPtr nearest_lane;

  double nearest_s;
  double nearest_l;

  int status =
      hdmap->GetNearestLane(point_enu, &nearest_lane, &nearest_s, &nearest_l);
  // TODO(lizh): make it a formal status below
  if (status != 0) {
    AERROR << "Failed to get nearest lane for point " << point.DebugString();
    return -1.0;
  }
  AINFO << "Dist: " << nearest_l;
  return nearest_l;
}

double Speed(const Point& point) {
  return std::sqrt(point.x() * point.x() + point.y() + point.y());
}

double Speed(const double vx, const double vy) {
  return std::sqrt(vx * vx + vy * vy);
}

double HeadingDifference(const double theta1, const double theta2) {
  double theta_diff = std::abs(theta1 - theta2);
  return theta_diff > M_PI ? (2 * M_PI - theta_diff) : theta_diff;
}

}  // namespace third_party_perception
}  // namespace apollo
