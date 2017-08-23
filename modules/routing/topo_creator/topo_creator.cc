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

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/topo_creator/graph_creator.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::unique_ptr<::apollo::routing::GraphCreator> creator_ptr;
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  creator_ptr.reset(new ::apollo::routing::GraphCreator(
      apollo::hdmap::BaseMapFile(), routing_map_file));
  CHECK(creator_ptr->Create()) << "Create routing topo failed!";
  AINFO << "Create routing topo successfully from " << routing_map_file;
  return 0;
}
