/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#include <memory>

#include <gflags/gflags.h>

#include "cyber/common/log.h"
#include "cyber/init.h"
#include "modules/contrib/cyber_bridge/node.h"
#include "modules/contrib/cyber_bridge/server.h"

// bazel build //cyber/bridge:cyber_bridge
// GLOG_v=4 GLOG_logtostderr=1
// ./bazel-bin/modules/contrib/cyber_bridge/cyber_bridge

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::cyber::Init(argv[0]);

  {
    Node node;

    auto server = std::make_shared<Server>(&node);
    server->run();
  }

  apollo::cyber::Clear();
}
