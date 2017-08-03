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

#include <memory>

#include "gflags/gflags.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/topo_creator/graph_creator.h"
#include "modules/common/log.h"

DEFINE_string(base_map_dir, "/apollo/modules/map/data", "directory of base map");
DEFINE_string(base_map_name, "base_map.txt", "file name of base map");
DEFINE_string(dump_topo_path, "/apollo/modules/map/data/routing_map.bin", "dump path of routing topology file");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::unique_ptr<::apollo::routing::GraphCreator> creator_ptr;
  creator_ptr.reset(new ::apollo::routing::GraphCreator(
                                FLAGS_base_map_dir + "/" + FLAGS_base_map_name,
                                FLAGS_dump_topo_path));
  if (!creator_ptr->create()) {
    AERROR << "Create router topo failed!";
    return -1;
  }
  return 0;
}

