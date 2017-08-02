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

#include <string>

#include "gtest/gtest.h"
#include "modules/routing/common/utils.h"

#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

TEST(FileUtilsTestSuit, test1) {
  std::string file_path = "./test.bin";
  std::string map_version = "versionA";

  ::apollo::routing::common::Graph graph_1;
  graph_1.set_hdmap_version(map_version);

  ::apollo::routing::common::Graph graph_2;
  ASSERT_TRUE(FileUtils::dump_protobuf_data_to_file(file_path, graph_1));
  ASSERT_TRUE(FileUtils::load_protobuf_data_from_file(file_path, &graph_2));
  ASSERT_EQ(map_version, graph_2.hdmap_version());
}

}  // namespace routing
}  // namespace apollo

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
