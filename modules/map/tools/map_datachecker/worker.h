/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker.h
 * @desc A worker for demo show, create grpc channel between app.
 * @author Tong Wu<wutong14@baidu.com>
 *****************************************************************************/

#ifndef _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_H
#define _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_H
#include <grpc++/grpc++.h>
#include <string>

namespace adu {
namespace workers {
namespace collection {

class Mapdatachecker {
 public:
    Mapdatachecker() {}
    ~Mapdatachecker() {}

    bool Init();
    bool Start();
    bool Stop();
    void Report();

 private:
    std::string _grpc_address;
};

}  // namespace collection
}  // namespace workers
}  // namespace adu

#endif  // _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_H
