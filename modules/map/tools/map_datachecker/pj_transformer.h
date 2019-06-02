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

#include "proj_api.h"

namespace adu {
namespace workers {
namespace collection {

class PJTransformer {
public:
    PJTransformer(int zone_id = 50);
    ~PJTransformer();
    int latlong_to_utm(long point_count, int point_offset, double *x, double *y, double *z);
private:
    projPJ _pj_latlong;
    projPJ _pj_utm;
};

} // collection
} // workers
} // adu


#endif // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_PJ_TRANSFORMER_H 
