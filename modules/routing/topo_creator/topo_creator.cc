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

// 处理文件读写
#include "cyber/common/file.h"
// 处理高精地图相关的工具类
#include "modules/map/hdmap/hdmap_util.h"
// 路由模块中定义的命令行参数（gflags）
#include "modules/routing/common/routing_gflags.h"
// 创建路由拓扑图的核心逻辑
#include "modules/routing/topo_creator/graph_creator.h"

int main(int argc, char **argv) {
  // 初始化 Google 日志系统，用程序名作为日志标签
  google::InitGoogleLogging(argv[0]);
  // 解析命令行参数并赋值给 GFlags 变量，例如 --routing_conf_file
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::routing::RoutingConfig routing_conf;

  ACHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_routing_conf_file,
                                                 &routing_conf))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

// base_map: 基础地图路径
  const auto base_map = apollo::hdmap::BaseMapFile();
// routing_map: 路由地图路径（输出路径）
  const auto routing_map = apollo::hdmap::RoutingMapFile();
  
// 使用基础地图路径、输出路径、配置参数构造一个 GraphCreator 对象
  apollo::routing::GraphCreator creator(base_map, routing_map, routing_conf);
// 生成路由拓扑图
  ACHECK(creator.Create()) << "Create routing topo failed!";

  AINFO << "Create routing topo successfully from " << base_map << " to "
        << routing_map;
  return 0;
}
