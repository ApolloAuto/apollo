/******************************************************************************
 * Created on Sat Jan 12 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_STATIC_ALIGN_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_STATIC_ALIGN_H

#include <memory>
#include "common.hpp"
#include "alignment.hpp"

namespace adu {
namespace workers {
namespace collection {

enum class StaticAlignDetectMethod {
    RANSAC,
    DYNAMIC_CENTROID,
};


typedef struct Point3d {
    Point3d():x(0.0), y(0.0), z(0.0) {}
    double x, y, z; 
} Point3d;
typedef struct Centroid3D {
    Centroid3D(): count(0), start_time(0.0), end_time(0.0) {}
    Point3d center;
    int count;
    double start_time, end_time;
} Centroid3D;


class StaticAlign: public Alignment {
public:
    StaticAlign(std::shared_ptr<JSonConf> sp_conf);
    ErrorCode process(std::vector<FramePose>& poses);

private:
    void reset();
    double get_static_align_progress(std::vector<FramePose> & poses);
    double static_align_ransac(std::vector<FramePose> & poses);
    double static_align_dynamic_centroid(std::vector<FramePose> & poses);
    //bool is_bad_pose_overload();
    double get_centroid_time_during();
    void update_dynamic_centroid(FramePose& pose);
    //int select_first_good_pose(std::vector<FramePose> & poses);
    bool is_static_pose(FramePose& pose);
    void update_good_pose_info(FramePose& pose);

private:
    StaticAlignDetectMethod _static_align_detect_method;
    Centroid3D _dynamic_centroid;
};


} // collection
} // workers
} // adu

#endif // #ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_STATIC_ALIGN_H
