/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file main.cpp
 * @desc The main function of demo-worker.
 * @author Tong Wu<wutong14@baidu.com>
 *****************************************************************************/

#include <google/protobuf/text_format.h>
#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/worker.h"


int main(int argc, char** argv) {
    fprintf(stderr, "parsing command lines\n");
    google::ParseCommandLineFlags(&argc, &argv, true);
    fprintf(stdout, "parsing command lines done\n");

    fprintf(stderr, "init logger\n");
    if (apollo::cyber::Init(argv[0])) {
        AINFO << "init logger succeed";
    } else {
        fprintf(stderr, "init logger failed\n");
    }

    google::SetStderrLogging(FLAGS_minloglevel);

    AINFO << "starting worker";
    adu::workers::collection::Mapdatachecker worker;
    if (!worker.Start()) {
        AFATAL << "Start Mapdatachecker Failed!";
        return -1;
    }

    apollo::cyber::WaitForShutdown();
    return 0;
}
