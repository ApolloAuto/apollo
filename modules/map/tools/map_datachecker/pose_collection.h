/******************************************************************************
 * Created on Thu Aug 16 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file pose_collect.h:
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_H
#include <memory>
#include <vector>
#include "modules/map/tools/map_datachecker/common.hpp"

namespace adu {
namespace workers {
namespace collection {

enum class PoseCollectionState {
    IDLE,
    RUNNING
};

class PoseCollection {
 public:
    explicit PoseCollection(std::shared_ptr<JSonConf> sp_conf);
    void collect(const FramePose& pose);
    std::shared_ptr<std::vector<FramePose>> get_poses();

 private:
    void reset();

 private:
    std::shared_ptr<std::vector<FramePose>> _sp_poses = nullptr;
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
};

}  // namespace collection
}  // namespace workers
}  // namespace adu


#endif  // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_H
