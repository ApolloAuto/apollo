/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker_cyber_node.h
 * @desc A cybertron node for demo-worker.
 * @author Tong Wu<wutong14@baidu.com>, yuanyijun@baidu.com
 *****************************************************************************/

#ifndef _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_CYBER_NODE_H
#define _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_CYBER_NODE_H

#include <memory>
#include "cyber/cyber.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"

namespace adu {
namespace workers {
namespace collection {

class MapDataCheckerAgent;

// need to add cybertron node creater
class MapDataCheckerCyberNode:
    public std::enable_shared_from_this<MapDataCheckerCyberNode> {
 public:
    MapDataCheckerCyberNode(
        std::shared_ptr<MapDataCheckerAgent> agent,
        bool *init_success);

    inline std::shared_ptr<MapDataCheckerCyberNode> get_worker_cyber_node() {
        return shared_from_this();
    }

 private:
    int create_channel_subscriber();

 private:
    using GnssBestPose_t = apollo::drivers::gnss::GnssBestPose;
    std::shared_ptr<apollo::cyber::Node> _node = nullptr;
    std::shared_ptr<apollo::cyber::Reader<GnssBestPose_t>>
        _bestgnsspos_reader = nullptr;
    std::shared_ptr<MapDataCheckerAgent> _agent = nullptr;
};

}  // namespace collection
}  // namespace workers
}  // namespace adu

#endif  // _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_CYBER_NODE_H
