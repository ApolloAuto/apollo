/******************************************************************************
 * Created on Thu Aug 16 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file: pose_collection.cpp
 * @desc: description
 * @author: author
  *****************************************************************************/
#include "modules/map/tools/map_datachecker/pose_collection.h"

namespace adu {
namespace workers {
namespace collection {

PoseCollection::PoseCollection(std::shared_ptr<JSonConf> sp_conf) {
    _sp_conf = sp_conf;
    reset();
}

void PoseCollection::reset() {
    _sp_poses = std::make_shared<std::vector<FramePose>>();
}

void PoseCollection::collect(const FramePose& pose) {
    if (_sp_poses == nullptr) {
        _sp_poses = std::make_shared<std::vector<FramePose>>();
    }
    _sp_poses->push_back(pose);
}

std::shared_ptr<std::vector<FramePose>> PoseCollection::get_poses() {
    return _sp_poses;
}

}  // namespace collection
}  // namespace workers
}  // namespace adu
