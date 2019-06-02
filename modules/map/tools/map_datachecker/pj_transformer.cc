/******************************************************************************
 * Created on Thu Aug 16 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file: common.h
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/

#include <iostream>
#include <sstream>
#include "pj_transformer.h"

namespace adu {
namespace workers {
namespace collection {


PJTransformer::PJTransformer(int zone_id) {
    // init projPJ
    std::stringstream stream;
    stream << "+proj=utm +zone=" << zone_id << " +ellps=WGS84" << std::endl;
    _pj_utm = pj_init_plus(stream.str().c_str());
    if (_pj_utm == NULL) {
        std::cerr << "proj4 init failed!" << stream.str() << std::endl;
        return;
    }
    _pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84");
    if (_pj_latlong == NULL) {
        std::cerr << "proj4 pj_latlong init failed!";
        return;
    }
    std::cerr << "proj4 init success" << std::endl;
}

PJTransformer::~PJTransformer() {
    if (_pj_latlong) {
        pj_free(_pj_latlong);
        _pj_latlong = NULL;
    }
    if (_pj_utm) {
        pj_free(_pj_utm);
        _pj_utm = NULL;
    }
}
int PJTransformer::latlong_to_utm(long point_count, int point_offset,
                                  double *x, double *y, double *z) { 
    if (!_pj_latlong || !_pj_utm) {
        std::cerr << "_pj_latlong:" << _pj_latlong << "_pj_utm:" << _pj_utm << std::endl;
        return -1;
    }
    return pj_transform(_pj_latlong, _pj_utm, point_count, point_offset, x, y, z);
}


}  // adu
}  // workers
}  // collection
