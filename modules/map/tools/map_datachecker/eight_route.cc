/******************************************************************************
 * Created on Thu Aug 16 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: author
  *****************************************************************************/

#include "modules/map/tools/map_datachecker/eight_route.h"
#include <cmath>
#include <vector>

namespace adu {
namespace workers {
namespace collection {

EightRoute::EightRoute(std::shared_ptr<JSonConf> sp_conf): Alignment(sp_conf) {
    reset();
}

void EightRoute::reset() {
    _progress = 0.0, _last_progress = 0;
}

bool EightRoute::is_eight_route_pose(
    const std::vector<FramePose> & poses, int pose_index) {
    if (poses.size() == 0 ||
        pose_index <= 0 ||
        pose_index >= static_cast<int>(poses.size())) {
        AINFO << "params error, poses size: " << poses.size()
              << ", pose_index: " << pose_index;
        return true;
    }

    double yaw = get_yaw(poses[pose_index-1].tx,
        poses[pose_index-1].ty, poses[pose_index].tx, poses[pose_index].ty);
    double yaw_diff = std::abs(_last_yaw - yaw);
    _last_yaw = yaw;
    yaw_diff = yaw_diff < 180 ? yaw_diff : 360 - yaw_diff;

    double xdiff = poses[pose_index].tx - poses[pose_index-1].tx;
    double ydiff = poses[pose_index].ty - poses[pose_index-1].ty;
    double zdiff = poses[pose_index].tz - poses[pose_index-1].tz;
    double dist = std::sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff);
    double during =
        poses[pose_index].time_stamp - poses[pose_index-1].time_stamp;
    if (during < 0) {
        AINFO << "skip back pose is bad pose";
        return false;
    }
    double vel = dist / during;
    AINFO << std::to_string(poses[pose_index].time_stamp)
          << ", yaw_diff:" << yaw_diff
          << ", dist: " << dist
          << ", during: " << during
          << ", vel: " << vel;
    if (yaw_diff > _sp_conf->eight_angle && vel > _sp_conf->eight_vel) {
        return true;
    }
    return false;
}

double EightRoute::get_good_pose_during() {
    if (_sp_good_pose_info->start_time < 0
        || _sp_good_pose_info->end_time < 0) {
        return 0.0;
    }
    return _sp_good_pose_info->end_time - _sp_good_pose_info->start_time;
}

double EightRoute::get_eight_route_progress(
    const std::vector<FramePose> & poses) {
    // double hz = get_hz(poses);
    int size = static_cast<int>(poses.size());
    int start_index = time_to_index(poses, _start_time);
    // select first good pose
    while (start_index < size) {
        if (is_good_pose(poses, start_index) &&
            is_eight_route_pose(poses, start_index)) {
            AINFO << "find first good pose.index:" << start_index;
            break;
        }
        start_index++;
    }
    if (start_index >= size) {
        AINFO << "not find first good pose, start_time: "
              << std::to_string(_start_time)
              << ", start_index: "<< start_index
              << ", pose size: " << size;
        return 0.0;
    }
    if (start_index + 1 >= size) {
        AINFO << "not have enough poses, wait for a moment";
        return 0.0;
    }
    _last_yaw = get_yaw(poses[start_index].tx,
        poses[start_index].ty,
        poses[start_index+1].tx,
        poses[start_index+1].ty);

    int not_eight_count = 0;
    for (int i = start_index + 2; i < size; i++) {
        if (!is_good_pose(poses, i)) {
            AINFO << "not good pose";
            return 0.0;
        }
        if (!is_eight_route_pose(poses, i)) {
            not_eight_count++;
            AINFO << "not eight route pose";
            if (not_eight_count > _sp_conf->eight_bad_pose_tolerance) {
                AINFO << "not-eight pose count reached upper limitation";
                _return_state = ErrorCode::ERROR_NOT_EIGHT_ROUTE;
                return 0.0;
            }
        } else {
            not_eight_count = 0;
        }
        AINFO << "good pose";
        update_good_pose_info(poses[i]);
    //    clear_bad_pose_info();
    }
    double eight_route_during = get_good_pose_during();
    if (eight_route_during < 1e-8) {
        AINFO << "num of eight route good pose too small, during: "
              << eight_route_during;
        _return_state = ErrorCode::SUCCESS;
        return 0.0;
    }
    _return_state = ErrorCode::SUCCESS;
    double progress = eight_route_during / _sp_conf->eight_duration;
    if (progress >= 1.0) {
        progress = 1.0;
    }
    clear_good_pose_info();
    return progress;
}

ErrorCode EightRoute::process(const std::vector<FramePose>& poses) {
    AINFO << "[EightRoute::process] begin";
    size_t size = poses.size();
    if (size <= 1) {
        _return_state = ErrorCode::ERROR_VERIFY_NO_GNSSPOS;
        return _return_state;
    }

    _progress = get_eight_route_progress(poses);
    if (_return_state != ErrorCode::SUCCESS) {
        AINFO << "get_eight_route_progress failed.";
        return _return_state;
    }
    if (_progress < _last_progress) {
        _return_state = ErrorCode::ERROR_NOT_EIGHT_ROUTE;
        return _return_state;
    }

    AINFO << "[EightRoute::process] end, progress:" << _progress;
    _return_state = ErrorCode::SUCCESS;
    return _return_state;
}

double EightRoute::get_progress() {
    return _progress;
}

}  // namespace collection
}  // namespace workers
}  // namespace adu
