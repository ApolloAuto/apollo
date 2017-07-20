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

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/map/proto/map.pb.h"

#include <string>

DEFINE_string(map_file, "modules/map/data/base_map.txt", "map file");
DEFINE_double(x_offset, 352904.810943, "x offset");
DEFINE_double(y_offset, 4142355.44248, "y offset");
DEFINE_string(output_dir, "/tmp/", "output map directory");

using apollo::hdmap::Map;

void ShiftMap(Map& map_pb) {
  for (auto& lane : *(map_pb.mutable_lane())) {
    for (auto& segment : *(lane.mutable_central_curve()->mutable_segment())) {
      for (auto& point : *(segment.mutable_line_segment()->mutable_point())) {
        point.set_x(point.x() + FLAGS_x_offset);
        point.set_y(point.y() + FLAGS_y_offset);
      }
    }
  }
  for (auto& stop_sign : *(map_pb.mutable_stop_sign())) {
    for (auto& segment : *(stop_sign.mutable_stop_line()->mutable_segment())) {
      for (auto& point : *(segment.mutable_line_segment()->mutable_point())) {
        point.set_x(point.x() + FLAGS_x_offset);
        point.set_y(point.y() + FLAGS_y_offset);
      }
    }
  }
}

void OutputMap(const Map& map_pb) {
  std::ofstream map_txt_file(FLAGS_output_dir + "/base_map.txt");
  map_txt_file << map_pb.DebugString();
  map_txt_file.close();

  std::ofstream map_bin_file(FLAGS_output_dir + "/base_map.bin");
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

  if (!::apollo::common::util::GetProtoFromFile(FLAGS_map_file,
                                                &map_pb)) {
    AERROR << "Fail to open:" << FLAGS_map_file;
    return 1;
  }
  ShiftMap(map_pb);
  OutputMap(map_pb);
  AINFO << "modified map at:" << FLAGS_output_dir;
}
