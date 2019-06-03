/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker_cyber_node.cpp
 * @desc A cybertron node for demo-worker.
 * @author Tong Wu<wutong14@baidu.com>
 *****************************************************************************/
#include "modules/map/tools/map_datachecker/worker_cyber_node.h"
#include "modules/map/tools/map_datachecker/worker_agent.h"
#include "modules/map/tools/map_datachecker/worker_gflags.h"
#include <memory>
#include <string>

constexpr double RADIANS_TO_DEGREES = 180.0 / M_PI;
constexpr double DEGRESS_TO_RADIANS = M_PI / 180.0;

namespace adu {
namespace workers {
namespace collection {

MapDataCheckerCyberNode::MapDataCheckerCyberNode(
    std::shared_ptr<MapDataCheckerAgent> agent, bool *init_success) {
    if (!agent) {
        AFATAL << "MapDataCheckerAgent pointer is nullptr";
        *init_success = false;
        return;
    }

    _agent = agent->get_worker_agent();
    _node = apollo::cyber::CreateNode(
        std::string("cybernode_map_datachecker"));
    if (!_node) {
        AFATAL << "Create cybertron node failed.";
        *init_success = false;
        return;
    }

    // Readers
    create_channel_subscriber();

    *init_success = true;
    AINFO << "map-datachecker cyber node create successfully";
}

// MapDataCheckerCyberNode::~MapDataCheckerCyberNode() {
//     //delete _pj_transformer;
// }

int MapDataCheckerCyberNode::create_channel_subscriber() {
    AINFO << "create bestgnsspos reader, topic: " << FLAGS_topic_bestgnsspos;
    _bestgnsspos_reader = _node->CreateReader<GnssBestPose_t> (
        FLAGS_topic_bestgnsspos,
        [this](const std::shared_ptr<const GnssBestPose_t> &msg) {
            _agent->
            get_sp_pose_collection_agent()->
            on_bestgnsspos_callback(msg);
        });
    if (!_bestgnsspos_reader) {
        AFATAL << "create bestgnsspos reader error";
        return -1;
    }
    return 0;
}

}  // namespace collection
}  // namespace workers
}  // namespace adu
