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

#include "modules/common/util/file.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map/proto/map.pb.h"

int main(int argc, char **argv) {
  CHECK_GE(argc, 2) << "missing file path.";

  const std::string map_filename = argv[1];
  apollo::hdmap::Map pb_map;
  CHECK(apollo::hdmap::adapter::OpendriveAdapter::LoadData(
      map_filename, &pb_map)) << "fail to load data";

  const std::string output_ascii_file = map_filename + ".txt";
  CHECK(apollo::common::util::SetProtoToASCIIFile(pb_map, output_ascii_file));

  const std::string output_bin_file = map_filename + ".bin";
  CHECK(apollo::common::util::SetProtoToBinaryFile(pb_map, output_bin_file));

  pb_map.Clear();
  CHECK(apollo::common::util::GetProtoFromFile(output_bin_file, &pb_map))
      << "load map fail";

  AINFO << "load map success";
  return 0;
}
