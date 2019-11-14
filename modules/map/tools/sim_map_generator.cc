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

#include "absl/strings/match.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/points_downsampler.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/proto/map.pb.h"

/**
 * A map tool to generate a downsampled map to be displayed by dreamview
 * frontend.
 */

DEFINE_string(output_dir, "/tmp/", "output map directory");
DEFINE_double(angle_threshold, 1. / 180 * M_PI, /* 1 degree */
              "Points are sampled when the accumulated direction change "
              "exceeds the threshold");
DEFINE_int32(downsample_distance, 5, "downsample rate for a normal path");
DEFINE_int32(steep_turn_downsample_distance, 1,
             "downsample rate for a steep turn path");

using apollo::common::PointENU;
using apollo::common::util::DownsampleByAngle;
using apollo::common::util::DownsampleByDistance;
using apollo::cyber::common::GetProtoFromFile;
using apollo::hdmap::Curve;
using apollo::hdmap::Map;
using apollo::hdmap::adapter::OpendriveAdapter;

static void DownsampleCurve(Curve* curve) {
  auto* line_segment = curve->mutable_segment(0)->mutable_line_segment();
  std::vector<PointENU> points(line_segment->point().begin(),
                               line_segment->point().end());
  line_segment->clear_point();

  // NOTE: this not the most efficient implementation, but since this map tool
  // is only run once for each, we can probably live with that.

  // Downsample points by angle then by distance.
  auto sampled_indices = DownsampleByAngle(points, FLAGS_angle_threshold);
  std::vector<PointENU> downsampled_points;
  for (const size_t index : sampled_indices) {
    downsampled_points.push_back(points[index]);
  }

  sampled_indices =
      DownsampleByDistance(downsampled_points, FLAGS_downsample_distance,
                           FLAGS_steep_turn_downsample_distance);

  for (const size_t index : sampled_indices) {
    *line_segment->add_point() = downsampled_points[index];
  }
  size_t new_size = line_segment->point_size();
  CHECK_GT(new_size, 1);

  AINFO << "Lane curve downsampled from " << points.size() << " points to "
        << new_size << " points.";
}

static void DownsampleMap(Map* map_pb) {
  for (int i = 0; i < map_pb->lane_size(); ++i) {
    auto* lane = map_pb->mutable_lane(i);
    lane->clear_left_sample();
    lane->clear_right_sample();
    lane->clear_left_road_sample();
    lane->clear_right_road_sample();

    AINFO << "Downsampling lane " << lane->id().id();
    DownsampleCurve(lane->mutable_central_curve());
    DownsampleCurve(lane->mutable_left_boundary()->mutable_curve());
    DownsampleCurve(lane->mutable_right_boundary()->mutable_curve());
  }
}

static void OutputMap(const Map& map_pb) {
  std::ofstream map_txt_file(FLAGS_output_dir + "/sim_map.txt");
  map_txt_file << map_pb.DebugString();
  map_txt_file.close();

  std::ofstream map_bin_file(FLAGS_output_dir + "/sim_map.bin");
  std::string map_str;
  map_pb.SerializeToString(&map_str);
  map_bin_file << map_str;
  map_bin_file.close();
}

int main(int32_t argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  Map map_pb;
  const auto map_file = apollo::hdmap::BaseMapFile();
  if (absl::EndsWith(map_file, ".xml")) {
    CHECK(OpendriveAdapter::LoadData(map_file, &map_pb));
  } else {
    CHECK(GetProtoFromFile(map_file, &map_pb)) << "Fail to open: " << map_file;
  }

  DownsampleMap(&map_pb);
  OutputMap(map_pb);
  AINFO << "sim_map generated at:" << FLAGS_output_dir;

  return 0;
}
