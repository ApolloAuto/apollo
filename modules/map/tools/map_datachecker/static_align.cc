/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/map/tools/map_datachecker/static_align.h"
#include <vector>

namespace apollo {
namespace hdmap {

StaticAlign::StaticAlign(
    std::shared_ptr<JSonConf> sp_conf): Alignment(sp_conf) {
    _sp_conf = sp_conf;
    _static_align_detect_method = StaticAlignDetectMethod::DYNAMIC_CENTROID;
    reset();
}

void StaticAlign::reset() {
    _progress = 0.0, _last_progress = 0.0;
    _start_time = _end_time = -1.0;
    _start_index = _end_index = -1;
    // _bad_pose_info = _good_pose_info = BadOrGoodPoseInfo();
    _sp_bad_pose_info = std::make_shared<BadOrGoodPoseInfo>();
    _sp_good_pose_info = std::make_shared<BadOrGoodPoseInfo>();
    _dynamic_centroid = Centroid3D();
}

bool StaticAlign::is_static_pose(const FramePose& pose) {
    if (_dynamic_centroid.count == 0) {
        return true;
    }
    double move_dist_x = pose.tx - _dynamic_centroid.center.x;
    double move_dist_y = pose.ty - _dynamic_centroid.center.y;
    double move_dist_z = pose.tz - _dynamic_centroid.center.z;
    double move_dist = std::sqrt(
        move_dist_x * move_dist_x
        + move_dist_y * move_dist_y
        + move_dist_z * move_dist_z);
    AINFO << "dist thresh: " << _sp_conf->static_align_dist_thresh
          << ", dist: " << move_dist;
    if (move_dist <= _sp_conf->static_align_dist_thresh) {
        return true;
    }
    return false;
}

void StaticAlign::update_dynamic_centroid(const FramePose& pose) {
    int count = _dynamic_centroid.count;
    if (count == 0) {
        _dynamic_centroid.start_time = pose.time_stamp;
    } else {
        _dynamic_centroid.end_time = pose.time_stamp;
    }
    AINFO << "cetroid start: " << _dynamic_centroid.start_time
          << ", end: " << _dynamic_centroid.end_time;

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
    if (_dynamic_centroid.start_time > 0 && _dynamic_centroid.end_time > 0) {
        return _dynamic_centroid.end_time - _dynamic_centroid.start_time;
    }
    return 0.0;
}

void StaticAlign::update_good_pose_info(const FramePose& pose) {
    update_dynamic_centroid(pose);
}

double StaticAlign::static_align_dynamic_centroid(
    const std::vector<FramePose> & poses) {
    int start_index = time_to_index(poses, _start_time);
    AINFO << "start_index:" << start_index << ",pose size:" << poses.size();
    _dynamic_centroid = Centroid3D();
    for (int i = start_index + 1; i < static_cast<int>(poses.size()); i++) {
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
        // clear_bad_pose_info();
        _return_state = ErrorCode::SUCCESS;
    }

    double progress =
        get_centroid_time_during() / _sp_conf->static_align_duration;
    if (progress > 1.0) {
        progress = 1.0;
    }
    return progress;
}

double StaticAlign::static_align_ransac(const std::vector<FramePose> & poses) {
    // TODO(yuanyijun): implementation of selecting an center by RANSAC
    return 0.0;
}

double StaticAlign::get_static_align_progress(
    const std::vector<FramePose> & poses) {
    double progress = 0.0;
    switch (_static_align_detect_method) {
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

ErrorCode StaticAlign::process(const std::vector<FramePose>& poses) {
    AINFO << "[StaticAlign::process] begin";
    size_t size = poses.size();
    if (size <= 1) {
        AINFO << "have no pose, exit process";
        _return_state = ErrorCode::ERROR_VERIFY_NO_GNSSPOS;
        return _return_state;
    }

    _progress = get_static_align_progress(poses);
    if (_return_state != ErrorCode::SUCCESS) {
        AINFO << "get_static_align_progress error, progress 0.0";
        return _return_state;
    }

    AINFO << "[StaticAlign::process] end, progress:" << _progress;
    _return_state = ErrorCode::SUCCESS;
    return _return_state;
}

}  // namespace hdmap
}  // namespace apollo
