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

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/proto/map_geometry.pb.h"

DEFINE_bool(xy_to_sl, false, "calculate xy to sl");
DEFINE_bool(sl_to_xy, false, "calculate sl to xy");
DEFINE_bool(xy_to_lane, false, "calculate xy to lane");
DEFINE_bool(lane_to_lane, false, "calculate lane to lane");
DEFINE_bool(dump_lane_width, false, "dump all lane width info");
DEFINE_string(dump_txt_map, "", "text file name for dumping map");
DEFINE_string(dump_bin_map, "", "binary file name for dumping map");
DEFINE_string(overlap, "", "get overlap information");
DEFINE_string(signal_info, "", "print signal info");
DEFINE_double(x, 0.0, "x");
DEFINE_double(y, 0.0, "y");
DEFINE_string(lane, "", "lane_id");
DEFINE_string(from_lane, "", "from_lane");
DEFINE_string(to_lane, "", "to_lane");
DEFINE_double(s, 0.0, "s");
DEFINE_double(l, 0.0, "l");

using apollo::common::PointENU;

namespace apollo {
namespace hdmap {

#define QUIT_IF(CONDITION, RET, LEVEL, MSG, ...) \
  do {                                           \
    if (CONDITION) {                             \
      RAW_LOG(LEVEL, MSG, ##__VA_ARGS__);        \
      return RET;                                \
    }                                            \
  } while (0);

std::ostream &operator<<(
    std::ostream &os,
    const ::google::protobuf::RepeatedPtrField<apollo::hdmap::Id> &ids) {
  for (int i = 0; i < ids.size(); ++i) {
    os << ids.Get(i).id();
    if (i != ids.size() - 1) {
      os << ", ";
    }
  }
  return os;
}

#define GET_ELEMENT_BY_ID(TYPE)                                     \
  const TYPE##InfoConstPtr Get##TYPE(const std::string &id) {       \
    auto ret = HDMapUtil::BaseMap().Get##TYPE##ById(MakeMapId(id)); \
    AERROR_IF(ret == nullptr)                                       \
        << "failed to find " << #TYPE << " with id: " << id;        \
    return ret;                                                     \
  }

class MapUtil {
 public:
  const OverlapInfo *GetOverlap(const std::string &overlap_id) const {
    auto ret = HDMapUtil::BaseMap().GetOverlapById(MakeMapId(overlap_id));
    AERROR_IF(ret == nullptr) << "failed to find overlap[" << overlap_id << "]";
    return ret.get();
  }

  GET_ELEMENT_BY_ID(ClearArea);
  GET_ELEMENT_BY_ID(Crosswalk);
  GET_ELEMENT_BY_ID(Junction);
  GET_ELEMENT_BY_ID(Lane);
  GET_ELEMENT_BY_ID(Signal);
  GET_ELEMENT_BY_ID(SpeedBump);
  GET_ELEMENT_BY_ID(StopSign);
  GET_ELEMENT_BY_ID(YieldSign);

  template <class T>
  void Print(const T &t) {
    std::cout << t.DebugString();
  }

  int PointToSL(const PointENU &point, std::string *lane_id, double *s,
                double *l, double *heading) const {
    QUIT_IF(lane_id == nullptr, -1, ERROR, "arg lane id is null");
    QUIT_IF(s == nullptr, -2, ERROR, "arg s is null");
    QUIT_IF(l == nullptr, -3, ERROR, "arg l is null");
    LaneInfoConstPtr lane = nullptr;
    int ret = HDMapUtil::BaseMap().GetNearestLane(point, &lane, s, l);
    QUIT_IF(ret != 0, -4, ERROR, "get_nearest_lane failed with ret[%d]", ret);
    QUIT_IF(lane == nullptr, -5, ERROR, "lane is null");
    *lane_id = lane->id().id();
    *heading = lane->Heading(*s);
    return 0;
  }

  int SLToPoint(LaneInfoConstPtr lane_ptr, const double s, const double l,
                PointENU *point, double *heading) const {
    QUIT_IF(point == nullptr, -1, ERROR, "arg point is null");
    QUIT_IF(heading == nullptr, -2, ERROR, "arg heading is null");
    QUIT_IF(lane_ptr == nullptr, -3, ERROR, "the provided lane_ptr is null");
    *point = lane_ptr->GetSmoothPoint(s);
    *heading = lane_ptr->Heading(s);
    auto normal_vec =
        common::math::Vec2d::CreateUnitVec2d(*heading + M_PI / 2.0) * l;
    point->set_x(point->x() + normal_vec.x());
    point->set_y(point->y() + normal_vec.y());

    return 0;
  }

  int LaneProjection(const apollo::common::math::Vec2d &vec2d,
                     const std::string &lane_id, double *s, double *l,
                     double *heading) const {
    QUIT_IF(s == nullptr, -1, ERROR, "arg s is nullptr");
    const auto lane = HDMapUtil::BaseMap().GetLaneById(MakeMapId(lane_id));
    QUIT_IF(lane == nullptr, -2, ERROR, "GetSignal_by_id[%s] failed",
            lane_id.c_str());
    bool ret = lane->GetProjection(vec2d, s, l);
    QUIT_IF(!ret, -3, ERROR, "lane[%s] get projection for point[%f, %f] failed",
            lane_id.c_str(), vec2d.x(), vec2d.y());
    *heading = lane->Heading(*s);
    return 0;
  }

  void PrintOverlap(const std::string &overlap_id) {
    const auto *overlap_ptr = GetOverlap(FLAGS_overlap);
    if (overlap_ptr == nullptr) {
      AERROR << "overlap_ptr is nullptr.";
      return;
    }
    ADEBUG << "overlap[" << overlap_ptr->id().id() << "] info["
           << overlap_ptr->overlap().DebugString() << "]" << std::endl;

    for (const auto &object_info : overlap_ptr->overlap().object()) {
      if (object_info.has_lane_overlap_info()) {
        std::cout << "Lane : " << object_info.id().id() << std::endl;
        PrintLane(GetLane(object_info.id().id()));
      } else if (object_info.has_signal_overlap_info()) {
        std::cout << "Signal: " << object_info.id().id() << std::endl;
        Print(GetSignal(object_info.id().id())->signal());
      } else if (object_info.has_stop_sign_overlap_info()) {
        std::cout << "StopSign: " << object_info.id().id() << std::endl;
        Print(GetStopSign(object_info.id().id())->stop_sign());
      } else if (object_info.has_crosswalk_overlap_info()) {
        std::cout << "Crosswalk: " << object_info.id().id() << std::endl;
        Print(GetCrosswalk(object_info.id().id())->crosswalk());
      } else if (object_info.has_junction_overlap_info()) {
        std::cout << "Junction: " << object_info.id().id() << std::endl;
        Print(GetJunction(object_info.id().id())->junction());
      } else if (object_info.has_yield_sign_overlap_info()) {
        std::cout << "YieldSign: " << object_info.id().id() << std::endl;
        Print(GetYieldSign(object_info.id().id())->yield_sign());
      } else if (object_info.has_clear_area_overlap_info()) {
        std::cout << "ClearArea: " << object_info.id().id() << std::endl;
        Print(GetClearArea(object_info.id().id())->clear_area());
      } else if (object_info.has_speed_bump_overlap_info()) {
        std::cout << "SpeedBump: " << object_info.id().id() << std::endl;
        Print(GetSpeedBump(object_info.id().id())->speed_bump());
      } else if (object_info.has_parking_space_overlap_info()) {
        std::cout << "ParkingSpace: " << object_info.id().id() << std::endl;
      } else {
        std::cout << "Unknown overlap type:  " << object_info.DebugString();
      }
    }
  }

  void PrintLane(const std::string &lane_id) { PrintLane(GetLane(lane_id)); }
  void PrintLane(LaneInfoConstPtr lane_ptr) {
    const auto &lane = lane_ptr->lane();
    PointENU start_point;
    double start_heading = 0.0;
    SLToPoint(lane_ptr, 0, 0, &start_point, &start_heading);

    PointENU end_point;
    double end_heading = 0.0;
    SLToPoint(lane_ptr, lane_ptr->total_length(), 0, &end_point, &end_heading);

    double left_width = 0.0;
    double right_width = 0.0;
    lane_ptr->GetWidth(FLAGS_s, &left_width, &right_width);

    std::cout << "lane[" << FLAGS_lane << std::fixed << "] length["
              << lane_ptr->total_length() << "] type["
              << Lane_LaneType_Name(lane.type()) << "] turn["
              << Lane_LaneTurn_Name(lane.turn()) << "] speed_limit["
              << lane.speed_limit() << "] predecessor[" << lane.predecessor_id()
              << "] successor[" << lane.successor_id() << "] left_forward["
              << lane.left_neighbor_forward_lane_id() << "] right_forward["
              << lane.right_neighbor_forward_lane_id() << "] left_reverse["
              << lane.left_neighbor_reverse_lane_id() << "] right_reverse["
              << lane.right_neighbor_reverse_lane_id() << "], "
              << "Left Boundary: [ virtual?:" << std::boolalpha
              << lane.left_boundary().virtual_() << ", Type: [";
    for (const auto &boundary_type : lane.left_boundary().boundary_type()) {
      std::cout << "s: " << boundary_type.s() << "->";
      for (const auto t : boundary_type.types()) {
        std::cout << LaneBoundaryType::Type_Name(
                         static_cast<LaneBoundaryType::Type>(t))
                  << ", ";
      }
    }

    std::cout << "]; Right Boundary: [ virtual?:" << std::boolalpha
              << lane.right_boundary().virtual_() << ", Type: ";
    for (const auto &boundary_type : lane.left_boundary().boundary_type()) {
      std::cout << "s: " << boundary_type.s() << "->";
      for (const auto t : boundary_type.types()) {
        std::cout << LaneBoundaryType::Type_Name(
                         static_cast<LaneBoundaryType::Type>(t))
                  << ", ";
      }
    }
    std::cout << "] overlap[" << lane.overlap_id() << "];"
              << " start point(x,y,heading):" << start_point.x() << ","
              << start_point.y() << "," << start_heading
              << " end point(x,y,heading):" << end_point.x() << ","
              << end_point.y() << "," << end_heading
              << " left_width:" << left_width << " right_width:" << right_width
              << std::endl;
    std::cout.unsetf(std::ios_base::fixed);

    if (FLAGS_dump_lane_width) {
      const auto sample_left_widthes = lane_ptr->sampled_left_width();
      std::cout << "left width num: " << sample_left_widthes.size()
                << std::endl;
      int num = 0;
      for (auto w : sample_left_widthes) {
        std::cout << " " << w.second;
        if (++num % 10 == 0) {
          std::cout << std::endl;
        }
      }
      std::cout << std::endl;
      num = 0;
      const auto sample_right_widthes = lane_ptr->sampled_right_width();
      std::cout << "right width num: " << sample_right_widthes.size()
                << std::endl;
      for (auto w : sample_right_widthes) {
        std::cout << " " << w.second;
        if (++num % 10 == 0) {
          std::cout << std::endl;
        }
      }
      std::cout << std::endl;
    }
  }
};  // namespace hdmap

}  // namespace hdmap
}  // namespace apollo

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  const std::string map_file = apollo::hdmap::BaseMapFile();
  bool valid_arg = false;

  apollo::hdmap::MapUtil map_util;

  if (FLAGS_xy_to_sl) {
    double x = FLAGS_x;
    double y = FLAGS_y;
    PointENU point;
    point.set_x(x);
    point.set_y(y);
    point.set_z(0);
    std::string lane_id;
    double s = 0.0;
    double l = 0.0;
    double heading = 0.0;
    map_util.PointToSL(point, &lane_id, &s, &l, &heading);
    printf("lane_id[%s], s[%f], l[%f], heading[%f]\n", lane_id.c_str(), s, l,
           heading);
    map_util.PrintLane(lane_id);
    valid_arg = true;
  }
  if (FLAGS_sl_to_xy) {
    PointENU point;
    double heading = 0.0;
    map_util.SLToPoint(map_util.GetLane(FLAGS_lane), FLAGS_s, FLAGS_l, &point,
                       &heading);
    printf("x[%f] y[%f], heading[%f]\n", point.x(), point.y(), heading);
    map_util.PrintLane(FLAGS_lane);
    valid_arg = true;
  }
  if (FLAGS_xy_to_lane) {
    double s = 0.0;
    double l = 0.0;
    double heading = 0.0;
    int ret = map_util.LaneProjection({FLAGS_x, FLAGS_y}, FLAGS_lane, &s, &l,
                                      &heading);
    if (ret != 0) {
      printf("lane_projection for x[%f], y[%f], lane_id[%s] failed\n", FLAGS_x,
             FLAGS_y, FLAGS_lane.c_str());
      return -1;
    }
    printf("lane[%s] s[%f], l[%f], heading[%f]\n", FLAGS_lane.c_str(), s, l,
           heading);
    map_util.PrintLane(FLAGS_lane);
    valid_arg = true;
  }
  if (FLAGS_lane_to_lane) {
    PointENU point;
    double src_heading = 0.0;
    map_util.SLToPoint(map_util.GetLane(FLAGS_from_lane), FLAGS_s, 0.0, &point,
                       &src_heading);
    double target_s = 0.0;
    double target_l = 0.0;
    double target_heading = 0.0;
    int ret = map_util.LaneProjection({point.x(), point.y()}, FLAGS_to_lane,
                                      &target_s, &target_l, &target_heading);
    if (ret != 0) {
      printf("lane_projection for lane[%s], s[%f] to lane_id[%s] failed\n",
             FLAGS_from_lane.c_str(), FLAGS_s, FLAGS_to_lane.c_str());
      return -1;
    }
    printf("lane[%s] s[%f], l[%f], heading[%f]\n", FLAGS_to_lane.c_str(),
           target_s, target_l, target_heading);
    map_util.PrintLane(FLAGS_from_lane);
    map_util.PrintLane(FLAGS_to_lane);
    valid_arg = true;
  }
  if (!FLAGS_lane.empty()) {
    const auto lane_ptr = map_util.GetLane(FLAGS_lane);
    if (!lane_ptr) {
      std::cout << "Could not find lane " << FLAGS_lane << " on map "
                << map_file;
      return 0;
    }
    map_util.PrintLane(lane_ptr);
    valid_arg = true;
  }
  if (!FLAGS_overlap.empty()) {
    map_util.PrintOverlap(FLAGS_overlap);
    valid_arg = true;
  }
  if (!FLAGS_signal_info.empty()) {
    std::cout << "Signal:  " << FLAGS_signal_info << std::endl;
    map_util.Print(map_util.GetSignal(FLAGS_signal_info)->signal());
    valid_arg = true;
  }
  if (!FLAGS_dump_txt_map.empty()) {
    apollo::hdmap::Map map;
    CHECK(apollo::cyber::common::GetProtoFromFile(map_file, &map));
    CHECK(apollo::cyber::common::SetProtoToASCIIFile(map, FLAGS_dump_txt_map));
    valid_arg = true;
  }
  if (!FLAGS_dump_bin_map.empty()) {
    apollo::hdmap::Map map;
    CHECK(apollo::cyber::common::GetProtoFromFile(map_file, &map));
    CHECK(apollo::cyber::common::SetProtoToBinaryFile(map, FLAGS_dump_bin_map));
    valid_arg = true;
  }
  if (!valid_arg) {
    std::cout << "usage: --map_dir map/file/directory/" << std::endl;
    std::cout << "usage: --base_map_filename map_file_name" << std::endl;
    std::cout << "usage: --dump_txt_map text_map_file" << std::endl;
    std::cout << "usage: --dump_bin_map bin_map_file" << std::endl;
    std::cout << "usage: --xy_to_sl --x x --y y" << std::endl;
    std::cout << "usage: --sl_to_xy --lane lane_id --s s --l l" << std::endl;
    std::cout << "usage: --xy_to_lane --x --y --lane" << std::endl;
    std::cout
        << "usage: --lane_to_lane --from_lane lane_id --s s --to_lane lane_id"
        << std::endl;
    std::cout << "usage: --lane lane_id" << std::endl;
    std::cout << "usage: --signal_info signal_id" << std::endl;
    std::cout << "usage: --overlap overlap_id" << std::endl;
  }
  return 0;
}
