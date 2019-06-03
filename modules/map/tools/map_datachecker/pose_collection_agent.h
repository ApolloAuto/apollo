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
#include <vector>
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/pose_collection.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/map/tools/map_datachecker/pj_transformer.h"

namespace adu {
namespace workers {
namespace collection {

class PoseCollectionAgent {
 public:
    explicit PoseCollectionAgent(std::shared_ptr<JSonConf> sp_conf);
    using GnssBestPose_t = const apollo::drivers::gnss::GnssBestPose;
    void on_bestgnsspos_callback(
        const std::shared_ptr<GnssBestPose_t> &bestgnsspos);
    std::shared_ptr<std::vector<FramePose>> get_poses();

 private:
    void reset();

 private:
    std::mutex _mutex;
    std::shared_ptr<PoseCollection> _sp_pose_collection = nullptr;
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<PJTransformer> _sp_pj_transformer = nullptr;
};

}  // namespace collection
}  // namespace workers
}  // namespace adu


#endif  // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_AGENT_H
