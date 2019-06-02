/******************************************************************************
 * Created on Thu Aug 16 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file eight_route.h:
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_EIGHT_ROUTE_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_EIGHT_ROUTE_H

#include <memory>
#include <grpc++/grpc++.h>
#include "cyber/cyber.h"
#include "common.hpp"
#include "worker_gflags.h"
#include "alignment.hpp"

namespace adu {
namespace workers {
namespace collection {


class EightRoute: public Alignment {

public:
    EightRoute(std::shared_ptr<JSonConf> sp_conf);
    ErrorCode process(std::vector<FramePose>& poses);
    double get_progress();

private:
    void reset();
    bool is_eight_route_pose(std::vector<FramePose> & poses, int pose_index);
    //double get_hz(std::vector<FramePose> & poses);
    //bool is_bad_pose_overload();
    double get_good_pose_during();
    double get_eight_route_progress(std::vector<FramePose> & poses);

private:
    double _progress;
    //char _start_count;
    double _last_yaw;
    //int _eight_tolerance;
    //int _valid_frame_count;
};


} // collection
} // workers
} // adu

#endif //_MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_EIGHT_ROUTE_H
