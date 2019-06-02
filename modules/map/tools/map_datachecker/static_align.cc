/******************************************************************************
 * Created on Sat Jan 12 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/

#include "static_align.h"

namespace adu {
namespace workers {
namespace collection {

StaticAlign::StaticAlign(std::shared_ptr<JSonConf> sp_conf): Alignment(sp_conf) {
    _sp_conf = sp_conf;
    _static_align_detect_method = StaticAlignDetectMethod::DYNAMIC_CENTROID;
    reset();
}

void StaticAlign::reset() {
    _progress = 0.0, _last_progress = 0.0;
    _start_time = _end_time = -1.0;
    _start_index = _end_index = -1;
    _bad_pose_info = _good_pose_info = BadOrGoodPoseInfo();
    _dynamic_centroid = Centroid3D();
}

bool StaticAlign::is_static_pose(FramePose& pose) {
    if (_dynamic_centroid.count == 0) {
        return true;
    }
    double move_dist_x = pose.tx - _dynamic_centroid.center.x;
    double move_dist_y = pose.ty - _dynamic_centroid.center.y;
    double move_dist_z = pose.tz - _dynamic_centroid.center.z;
    double move_dist = std::sqrt(move_dist_x * move_dist_x + move_dist_y * move_dist_y + move_dist_z * move_dist_z);
    AINFO << "dist thresh:" << _sp_conf->static_align_dist_thresh << ",dist:" << move_dist;
    if (move_dist <= _sp_conf->static_align_dist_thresh) {
        return true;
    }
    return false;
} 
/*
int StaticAlign::select_first_good_pose(std::vector<FramePose> & poses) {
    int start_index = 0;
    if ( _start_time > 0 ) {
        start_index = time_to_index(poses, _start_time);
    }
    for(size_t i = start_index; i < poses.size(); i++) {
        if ( is_good_pose(poses, i) ) {
            return i;
        } 
    }
    return -1;
}*/

void StaticAlign::update_dynamic_centroid(FramePose& pose) {
    int count = _dynamic_centroid.count;
    if ( count == 0 ) {
        _dynamic_centroid.start_time = pose.time_stamp;
    //    AINFO << "update start:" << _dynamic_centroid.start_time;
    } else {
        _dynamic_centroid.end_time = pose.time_stamp;
    //    AINFO << "update end:" << _dynamic_centroid.end_time;
    }
    //AINFO << "cetroid start:" << _dynamic_centroid.start_time << ", end:" << _dynamic_centroid.end_time;
    fprintf(stderr, "cetroid start:%lf, end:%lf\n", _dynamic_centroid.start_time, _dynamic_centroid.end_time);

    double x = _dynamic_centroid.center.x * count + pose.tx;
    double y = _dynamic_centroid.center.y * count + pose.ty;
    double z = _dynamic_centroid.center.z * count + pose.tz;
    count++;

    _dynamic_centroid.count = count;
    _dynamic_centroid.center.x = x / count;
    _dynamic_centroid.center.y = y / count;
    _dynamic_centroid.center.z = z / count;
}

double StaticAlign::get_centroid_time_during() {
    if ( _dynamic_centroid.start_time > 0 && _dynamic_centroid.end_time > 0 ) {
        return _dynamic_centroid.end_time - _dynamic_centroid.start_time;
    }
    return 0.0;
}

void StaticAlign::update_good_pose_info(FramePose& pose) {
    update_dynamic_centroid(pose);
}
/*
bool StaticAlign::is_bad_pose_overload() {
    if ( _bad_pose_info.end_time > 0 
         && _bad_pose_info.end_time - _bad_pose_info.start_time > _sp_conf->static_align_tolerance ) {
        return true;
    }
    return false;
}*/

double StaticAlign::static_align_dynamic_centroid(std::vector<FramePose> & poses) {
//    int first_good_pose = select_first_good_pose(poses);
//    if ( first_good_pose < 0 ) {
//        AINFO << "select_first_good_pose failed. pose count: " << poses.size() << ",index:" << first_good_pose;
//        return 0.0;
//    }
//    update_good_pose_info(poses[first_good_pose]);
//
    int start_index = time_to_index(poses, _start_time);
    AINFO << "start_index:" << start_index << ",pose size:" << poses.size();
    _dynamic_centroid = Centroid3D();
    for ( int i = start_index + 1; i < (int)poses.size(); i++ ) {
        if (!is_good_pose(poses, i)) {
            AINFO << "not good pose";
            _return_state = ErrorCode::ERROR_GNSS_SIGNAL_FAIL;
            return 0.0;
        }
        if (!is_static_pose(poses[i])) {
            AINFO << "not static pose";
            _return_state = ErrorCode::ERROR_NOT_STATIC;
            return 0.0;
        }
        update_good_pose_info(poses[i]);
//        clear_bad_pose_info();
        _return_state = ErrorCode::SUCCESS;
    }

    double progress = get_centroid_time_during() / _sp_conf->static_align_duration;
    if (progress > 1.0) {
        progress = 1.0;
    }
    return progress;
}

double StaticAlign::static_align_ransac(std::vector<FramePose> & poses) {
    // TODO
    return 0.0;
}

double StaticAlign::get_static_align_progress(std::vector<FramePose> & poses) {
    double progress = 0.0;
    switch(_static_align_detect_method) {
    case StaticAlignDetectMethod::DYNAMIC_CENTROID:
        progress = static_align_dynamic_centroid(poses);
        break;
    case StaticAlignDetectMethod::RANSAC:
        progress = static_align_ransac(poses);
        break;
    default:
        break;
    }
    clear_good_pose_info();
    return progress;
}

ErrorCode StaticAlign::process(std::vector<FramePose>& poses) {
    AINFO << "[StaticAlign::process] begin";
    size_t size = poses.size();
    if ( size <= 1 ) {
        AINFO << "have no pose, exit process";
        _return_state = ErrorCode::ERROR_VERIFY_NO_GNSSPOS;
        return _return_state;
    }

    _progress = get_static_align_progress(poses);
    if ( _return_state != ErrorCode::SUCCESS ) {
        AINFO << "get_static_align_progress error, progress 0.0";
        return _return_state;
    }

    AINFO << "[StaticAlign::process] end, progress:" << _progress;
    _return_state = ErrorCode::SUCCESS;
    return _return_state;
}



} // collection
} // workers
} // adu
