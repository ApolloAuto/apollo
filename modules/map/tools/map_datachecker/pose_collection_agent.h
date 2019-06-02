/******************************************************************************
 * Created on Fri Jan 11 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_AGENT_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_AGENT_H

#include <memory>
#include <mutex>
#include "common.hpp"
#include "pose_collection.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "pj_transformer.h"

namespace adu {
namespace workers {
namespace collection {

class PoseCollectionAgent {
public:
    PoseCollectionAgent(std::shared_ptr<JSonConf> sp_conf);
    void on_bestgnsspos_callback(const std::shared_ptr<const apollo::drivers::gnss::GnssBestPose> &bestgnsspos);
    std::shared_ptr<std::vector<FramePose>> get_poses();

private:
    void reset();

private:
    std::mutex _mutex;
    std::shared_ptr<PoseCollection> _sp_pose_collection = nullptr;
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<PJTransformer> _sp_pj_transformer = nullptr;
};


} // collection
} // workers
} // adu


#endif // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_AGENT_H
