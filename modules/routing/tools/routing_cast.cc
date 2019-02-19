/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/routing/proto/routing.pb.h"

DEFINE_string(routing_dump_file, "/tmp/routing.pb.txt",
              "file name to dump routing response.");

using apollo::cyber::Rate;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cyber::Init(argv[0]);

  std::shared_ptr<apollo::cyber::Node> cast_node(
      apollo::cyber::CreateNode("routing_cast"));

  apollo::routing::RoutingResponse routing_response;
  if (!apollo::cyber::common::GetProtoFromFile(FLAGS_routing_dump_file,
                                               &routing_response)) {
    AERROR << "failed to load file: " << FLAGS_routing_dump_file;
    return -1;
  }

  auto cast_writer = cast_node->CreateWriter<apollo::routing::RoutingResponse>(
      FLAGS_routing_response_topic);
  Rate rate(1.0);
  while (apollo::cyber::OK()) {
    apollo::common::util::FillHeader("routing", &routing_response);
    cast_writer->Write(routing_response);
    rate.Sleep();
  }
  return 0;
}
