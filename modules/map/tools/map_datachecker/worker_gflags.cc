/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker_gflags.cpp
 * @desc The gflags definition of demo-worker.
 * @author Tong Wu<wutong14@baidu.com>
 *****************************************************************************/


#include "modules/map/tools/map_datachecker/worker_gflags.h"

namespace adu {
namespace workers {
namespace collection {

// Server address
DEFINE_string(
    map_datachecker_host,
    "127.0.0.1",
    "the demo worker grpc server host");
DEFINE_string(
    map_datachecker_port,
    "50100",
    "the demo worker grpc server port");

// Cybertron topics
DEFINE_string(
    topic_bestgnsspos,
    "/apollo/sensor/gnss/best_pose",
    "the topic name for pose and status");
// configure file
DEFINE_string(
    conf_json,
    "/apollo/modules/map/tools/map_datachecker/conf/map-datachecker.json",
    "configure file");

}  // namespace collection
}  // namespace workers
}  // namespace adu
