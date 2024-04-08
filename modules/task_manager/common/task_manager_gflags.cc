/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/task_manager/common/task_manager_gflags.h"

DEFINE_string(task_manager_node_name, "task_manager", "the name for this node");

DEFINE_double(task_manager_threshold_for_destination_check, 1.0,
              "meters, which is 100 feet.  This threshold is used to check if"
              "the vehicle reaches the destination");

DEFINE_double(plot_size_buffer, 0.2, "the size buffer of parking plot");

DEFINE_double(road_width_buffer, 0.0, "the size buffer of road width");

DEFINE_double(search_junction_threshold, 1.0,
              "the threshold is used to search junction a certain range");
