/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker_gflags.h
 * @desc The gflags declaration of demo-worker.
 * @author Tong Wu<wutong14@baidu.com>
 *****************************************************************************/

#ifndef _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_GFLAGS_H
#define _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_GFLAGS_H

#include "gflags/gflags.h"

namespace adu {
namespace workers {
namespace collection {

// worker address
DECLARE_string(map_datachecker_host);
DECLARE_string(map_datachecker_port);

// hdmap file define
//DECLARE_string(demo_worker_base_map_filename);

// Cybertron topics
//DECLARE_string(demo_worker_adc_status_topic);
DECLARE_string(topic_inpspva);
DECLARE_string(topic_inpspvax);
DECLARE_string(topic_bestgnsspos);
DECLARE_string(conf_json);

}  // adu
}  // workers
}  // collection

#endif // _MODULES_DATA_CHECKER_DEMO_WORKER_INCLUDE_WORKER_GFLAGS_H
