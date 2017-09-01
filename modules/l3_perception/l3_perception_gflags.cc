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

#include "modules/l3_perception/l3_perception_gflags.h"

DEFINE_string(node_namespace, "/apollo/l3_perception", "Global node namespace");
DEFINE_string(node_name, "l3_perception", "The chassis module name in proto");
DEFINE_string(hmi_name, "l3_perception", "Module name in HMI");

DEFINE_double(l3_perception_freq, 100, "L3 perception timer frequency.");
