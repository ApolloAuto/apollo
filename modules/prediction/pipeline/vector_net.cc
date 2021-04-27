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

#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace prediction {
    bool query_nearby_map(const double obstacle_x,
                      const double obstacle_y,
                      const double obstacle_phi) {
        apollo::hdmap::HDMapUtil::ReloadMaps();
        return true;
    }
}
}

int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    apollo::prediction::query_nearby_map(Flag_obstacle_x, Flag_obstacle_y, Flag_obstacle_phi);
    return 0;
}

