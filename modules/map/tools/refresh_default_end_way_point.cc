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

// A tool to refresh default_end_way_point.txt file after the map is updated.
//
// Usage:
//  bazel-bin/modules/map/tools/refresh_default_end_way_point --map_dir=</yours>
//
// How it works:
//   Assuming that the lanes information is changed while our end point's
//   absolute (x,y,z) is still correct. Then we can find the nearest point on
//   the new map as the new end point.

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/common_msgs/routing_msgs/poi.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

namespace apollo {
namespace hdmap {

apollo::common::PointENU SLToXYZ(const std::string& lane_id, const double s,
                                 const double l) {
  const auto lane_info = HDMapUtil::BaseMap().GetLaneById(MakeMapId(lane_id));
  ACHECK(lane_info);
  return lane_info->GetSmoothPoint(s);
}

void XYZToSL(const apollo::common::PointENU& point, std::string* lane_id,
             double* s, double* l) {
  ACHECK(lane_id);
  ACHECK(s);
  ACHECK(l);
  LaneInfoConstPtr lane = nullptr;

  CHECK_EQ(HDMapUtil::BaseMap().GetNearestLane(point, &lane, s, l), 0);
  *lane_id = lane->id().id();
}

double XYZDistance(const apollo::common::PointENU& p1,
                   const apollo::common::PointENU& p2) {
  const double x_diff = p1.x() - p2.x();
  const double y_diff = p1.y() - p2.y();
  const double z_diff = p1.z() - p2.z();
  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

void RefreshDefaultEndPoint() {
  apollo::routing::POI old_poi;
  ACHECK(cyber::common::GetProtoFromASCIIFile(EndWayPointFile(), &old_poi));

  apollo::routing::POI new_poi;
  for (const auto& old_landmark : old_poi.landmark()) {
    apollo::routing::Landmark* new_landmark = new_poi.add_landmark();
    new_landmark->set_name(old_landmark.name());
    AINFO << "Refreshed point of interest: " << old_landmark.name();
    // Read xyz from old point.
    for (const auto& old_end_point : old_landmark.waypoint()) {
      apollo::common::PointENU old_xyz;
      old_xyz.set_x(old_end_point.pose().x());
      old_xyz.set_y(old_end_point.pose().y());
      old_xyz.set_z(old_end_point.pose().z());

      // Get new lane info from xyz.
      std::string new_lane;
      double new_s;
      double new_l;
      XYZToSL(old_xyz, &new_lane, &new_s, &new_l);

      // Get new xyz from lane info.
      const auto new_xyz = SLToXYZ(new_lane, new_s, new_l);

      // Update default end way point.
      apollo::routing::LaneWaypoint new_end_point;
      new_end_point.set_id(new_lane);
      new_end_point.set_s(new_s);
      auto* pose = new_end_point.mutable_pose();
      pose->set_x(new_xyz.x());
      pose->set_y(new_xyz.y());
      pose->set_z(new_xyz.z());
      *new_landmark->add_waypoint() = new_end_point;

      AINFO << "\n ============ from ============ \n"
            << old_end_point.DebugString()
            << "\n ============ to ============ \n"
            << new_end_point.DebugString() << "XYZ distance is "
            << XYZDistance(old_xyz, new_xyz);
    }
  }
  ACHECK(cyber::common::SetProtoToASCIIFile(new_poi, EndWayPointFile()));
}

}  // namespace hdmap
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  apollo::hdmap::RefreshDefaultEndPoint();
  return 0;
}
