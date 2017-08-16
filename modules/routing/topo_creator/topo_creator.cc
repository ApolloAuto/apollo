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
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/topo_creator/graph_creator.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::unique_ptr<::apollo::routing::GraphCreator> creator_ptr;
  creator_ptr.reset(new ::apollo::routing::GraphCreator(FLAGS_map_file_path,
                                                        FLAGS_graph_file_path));
  if (!creator_ptr->Create()) {
    AERROR << "Create routing topo failed!";
    return -1;
  }
  AINFO << "Create routing topo successfully from " << FLAGS_graph_file_path;
  return 0;
}
