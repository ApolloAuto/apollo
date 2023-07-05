/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "gflags/gflags.h"

#include "modules/common_msgs/map_msgs/map.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"

/**
 * A map tool to transform .txt map to .bin map
 */

DEFINE_string(output_dir, "/tmp", "output map directory");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, true);

  const auto map_filename = FLAGS_map_dir + "/base_map.txt";
  apollo::hdmap::Map pb_map;
  if (!apollo::cyber::common::GetProtoFromFile(map_filename, &pb_map)) {
    AERROR << "Failed to load txt map from " << map_filename;
    return -1;
  } else {
    AINFO << "Loaded txt map from " << map_filename;
  }

  const std::string output_bin_file = FLAGS_output_dir + "/base_map.bin";
  if (!apollo::cyber::common::SetProtoToBinaryFile(pb_map, output_bin_file)) {
    AERROR << "Failed to generate binary base map";
    return -1;
  }

  pb_map.Clear();
  ACHECK(apollo::cyber::common::GetProtoFromFile(output_bin_file, &pb_map))
      << "Failed to load generated binary base map";

  AINFO << "Successfully converted .txt map to .bin map: " << output_bin_file;

  return 0;
}
