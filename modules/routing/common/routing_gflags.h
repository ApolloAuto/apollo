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

#pragma once

#include "gflags/gflags.h"

DECLARE_string(routing_conf_file);  // Routing模块配置文件的路径
DECLARE_string(routing_node_name);  // Routing模块的节点名称

DECLARE_double(min_length_for_lane_change); // 在变道前，在当前车道上行驶的最短距离
DECLARE_bool(enable_change_lane_in_result); // 导航结果是否允许变道
DECLARE_uint32(routing_response_history_interval_ms); // 路由请求的响应时长
