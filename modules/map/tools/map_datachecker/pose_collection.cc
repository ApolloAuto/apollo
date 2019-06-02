/******************************************************************************
 * Created on Thu Aug 16 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file: pose_collection.cpp
 * @desc: description
 * @author: author
  *****************************************************************************/
#include "pose_collection.h"

namespace adu {
namespace workers {
namespace collection {

PoseCollection::PoseCollection(std::shared_ptr<JSonConf> sp_conf) {
    _sp_conf = sp_conf;
    reset();
}

void PoseCollection::reset() {
    _sp_poses = std::make_shared<std::vector<FramePose>>();
//    _state = PoseCollectionState::IDLE;
}

//void PoseCollection::set_state(PoseCollectionState state) {
//    _state = state;
//}

//PoseCollectionState PoseCollection::get_state() {
//    return _state;
//}

//void PoseCollection::start_collect() {
//    if ( get_state() == PoseCollectionState::RUNNING ) {
//        AINFO << "pose collection is working. do not need start again";
//        return;
//    }
//    set_state(PoseCollectionState::RUNNING);
//}

void PoseCollection::collect(FramePose& pose) {
//    if ( get_state() != PoseCollectionState::RUNNING ) {
//        AINFO << "pose collection is not work. it need start first";
//        return;
//    }

    if ( _sp_poses == nullptr ) {
        _sp_poses = std::make_shared<std::vector<FramePose>>();
    }
    _sp_poses->push_back(pose);
}

//void PoseCollection::stop_collect() {
//    set_state(PoseCollectionState::IDLE);
//}

std::shared_ptr<std::vector<FramePose>> PoseCollection::get_poses() {
    return _sp_poses;
}

} // collection
} // workers
} // adu
