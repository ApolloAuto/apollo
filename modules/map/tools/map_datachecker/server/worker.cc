/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/map/tools/map_datachecker/server/worker.h"
#include <grpc++/grpc++.h>
#include <memory>
#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/server/worker_agent.h"
#include "modules/map/tools/map_datachecker/server/worker_cyber_node.h"
#include "modules/map/tools/map_datachecker/server/worker_gflags.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;

namespace apollo {
namespace hdmap {

bool Mapdatachecker::Init() {
  _grpc_address = FLAGS_map_datachecker_host + ":" + FLAGS_map_datachecker_port;
  return true;
}

bool Mapdatachecker::Start() {
  AINFO << "Mapdatachecker::Start";
  Init();

  AINFO << "creating agent";
  std::shared_ptr<MapDataCheckerAgent> agent =
      std::make_shared<MapDataCheckerAgent>();

  AINFO << "creating node";
  bool cyber_node_inited = false;
  std::shared_ptr<MapDataCheckerCyberNode> cyber_node =
      std::make_shared<MapDataCheckerCyberNode>(agent, &cyber_node_inited);
  if (!cyber_node_inited) {
    AFATAL << "Error in create MapDataCheckerCyberNode";
    apollo::cyber::WaitForShutdown();
    apollo::cyber::Clear();
    return false;
  }
  agent->set_worker_cyber_node(cyber_node);

  AINFO << "register service";
  ServerBuilder builder;
  builder.AddListeningPort(_grpc_address, grpc::InsecureServerCredentials());
  builder.RegisterService(agent.get());
  std::unique_ptr<Server> server(builder.BuildAndStart());
  AINFO << "Server listening on " << _grpc_address;
  apollo::cyber::WaitForShutdown();
  apollo::cyber::Clear();
  return true;
}

bool Mapdatachecker::Stop() { return true; }

void Mapdatachecker::Report() {}

}  // namespace hdmap
}  // namespace apollo
