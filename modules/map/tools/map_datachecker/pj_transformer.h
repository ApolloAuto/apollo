/******************************************************************************
 * Created on Thu Aug 16 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: author
  *****************************************************************************/
#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_PJ_TRANSFORMER_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_PJ_TRANSFORMER_H

#include <proj_api.h>

namespace adu {
namespace workers {
namespace collection {

class PJTransformer {
 public:
    explicit PJTransformer(int zone_id = 50);
    ~PJTransformer();
    int latlong_to_utm(
        int64_t point_count, int point_offset, double *x, double *y, double *z);
 private:
    projPJ _pj_latlong;
    projPJ _pj_utm;
};

}  // namespace collection
}  // namespace workers
}  // namespace adu

#endif  // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_PJ_TRANSFORMER_H
