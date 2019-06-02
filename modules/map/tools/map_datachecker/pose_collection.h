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
#include "common.hpp"

namespace adu {
namespace workers {
namespace collection {

enum class PoseCollectionState {
    IDLE,
    RUNNING
};

class PoseCollection {
public:
    PoseCollection(std::shared_ptr<JSonConf> sp_conf);
//    void start_collect();
    void collect(FramePose& pose);
//    void stop_collect();
    std::shared_ptr<std::vector<FramePose>> get_poses();

private:
//    void set_state(PoseCollectionState state);
//    PoseCollectionState get_state();
    void reset();

private:
    std::shared_ptr<std::vector<FramePose>> _sp_poses = nullptr;
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
//    PoseCollectionState _state;
};


} // collection
} // workers
} // adu


#endif // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_POSE_COLLECTION_H
