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
#include <grpc++/grpc++.h>
#include <memory>
#include <vector>
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/worker_gflags.h"
#include "modules/map/tools/map_datachecker/alignment.hpp"
#include "cyber/cyber.h"

namespace adu {
namespace workers {
namespace collection {

class EightRoute: public Alignment {
 public:
    explicit EightRoute(std::shared_ptr<JSonConf> sp_conf);
    ErrorCode process(const std::vector<FramePose>& poses);
    double get_progress();

 private:
    void reset();
    bool is_eight_route_pose(
        const std::vector<FramePose> & poses, int pose_index);
    double get_good_pose_during();
    double get_eight_route_progress(const std::vector<FramePose> & poses);

 private:
    double _progress;
    double _last_yaw;
};

}  // namespace collection
}  // namespace workers
}  // namespace adu

#endif  // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_EIGHT_ROUTE_H
