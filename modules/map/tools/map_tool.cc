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
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/proto/map.pb.h"

DEFINE_double(x_offset, 587318.4866268333, "x offset");
DEFINE_double(y_offset, 4141146.110116891, "y offset");
DEFINE_string(output_dir, "/tmp/", "output map directory");

using apollo::hdmap::Map;

static void ShiftMap(Map* map_pb) {
  for (auto& lane : *(map_pb->mutable_lane())) {
    for (auto& segment : *(lane.mutable_central_curve()->mutable_segment())) {
      for (auto& point : *(segment.mutable_line_segment()->mutable_point())) {
        point.set_x(point.x() + FLAGS_x_offset);
        point.set_y(point.y() + FLAGS_y_offset);
      }
    }
    for (auto& segment :
         *(lane.mutable_left_boundary()->mutable_curve()->mutable_segment())) {
      for (auto& point : *(segment.mutable_line_segment()->mutable_point())) {
        point.set_x(point.x() + FLAGS_x_offset);
        point.set_y(point.y() + FLAGS_y_offset);
      }
    }
    for (auto& segment :
         *(lane.mutable_right_boundary()->mutable_curve()->mutable_segment())) {
      for (auto& point : *(segment.mutable_line_segment()->mutable_point())) {
        point.set_x(point.x() + FLAGS_x_offset);
        point.set_y(point.y() + FLAGS_y_offset);
      }
    }
  }
  for (auto& stop_sign : *(map_pb->mutable_stop_sign())) {
    for (auto& stop_line : *(stop_sign.mutable_stop_line())) {
      for (auto& segment : *(stop_line.mutable_segment())) {
        for (auto& point : *(segment.mutable_line_segment()->mutable_point())) {
          point.set_x(point.x() + FLAGS_x_offset);
          point.set_y(point.y() + FLAGS_y_offset);
        }
      }
    }
  }
}

static void OutputMap(const Map& map_pb) {
  const std::string txt_file = FLAGS_output_dir + "/base_map.txt";
  const std::string bin_file = FLAGS_output_dir + "/base_map.bin";
  CHECK(apollo::cyber::common::SetProtoToASCIIFile(map_pb, txt_file));
  CHECK(apollo::cyber::common::SetProtoToBinaryFile(map_pb, bin_file));
}

int main(int32_t argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  Map map_pb;
  const auto map_file = apollo::hdmap::BaseMapFile();
  CHECK(apollo::cyber::common::GetProtoFromFile(map_file, &map_pb))
      << "Fail to open:" << map_file;
  ShiftMap(&map_pb);
  OutputMap(map_pb);
  AINFO << "modified map at:" << FLAGS_output_dir;
}
