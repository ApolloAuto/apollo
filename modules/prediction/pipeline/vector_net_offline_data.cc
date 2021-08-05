/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include <fstream>
#include "modules/prediction/pipeline/vector_net.h"

using apollo::prediction::VectorNet;

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  VectorNet vector_net = VectorNet();

  // get world-coord from protobuf
  apollo::prediction::WorldCoord world_coords;
  std::fstream input(
      FLAGS_world_coordinate_file, std::ios::in | std::ios::binary);
  if (!world_coords.ParseFromIstream(&input)) {
    AERROR << "Failed to parse file: " << FLAGS_world_coordinate_file;
    return -1;
  }
  for (auto pose : world_coords.pose()) {
    double x = pose.x();
    double y = pose.y();
    double phi = pose.phi();
    std::string _file_name = \
        FLAGS_prediction_target_dir + "/" + pose.id() + ".pb.txt";
    vector_net.offline_query(x, y, phi, _file_name);
  }

  return 0;
}
