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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_ALIGNMENT_HPP
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_ALIGNMENT_HPP
#include <memory>
#include <vector>
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"

namespace apollo {
namespace hdmap {

typedef struct BadOrGoodPoseInfo {
    BadOrGoodPoseInfo(): start_time(-1.0), end_time(-1.0), pose_count(0) {}
    double start_time, end_time;
    int pose_count;
} BadOrGoodPoseInfo;

class Alignment {
 public:
    explicit Alignment(std::shared_ptr<JSonConf> sp_conf) {
        _return_state = ErrorCode::SUCCESS;
        _sp_conf = sp_conf;
    }
    virtual ~Alignment() {}
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    virtual ErrorCode process(const std::vector<FramePose>& poses) = 0;
    virtual void reset() = 0;

    virtual double get_progress() {
        return _progress;
    }

    virtual void set_start_time(double start_time) {
        _start_time = start_time;
    }

    virtual void set_end_time(double end_time) {
        _end_time = end_time;
    }

    virtual void update_bad_pose_info(const FramePose& pose) {
        update_pose_info(pose, _sp_bad_pose_info);
    }

    virtual void clear_bad_pose_info() {
        clear_pose_info(_sp_bad_pose_info);
    }

    virtual void update_good_pose_info(const FramePose& pose) {
        update_pose_info(pose, _sp_good_pose_info);
    }

    virtual void clear_good_pose_info() {
        clear_pose_info(_sp_good_pose_info);
    }

    virtual bool is_good_pose(
        const std::vector<FramePose> & poses, int pose_index) {
        if ( poses.size() < 0
            || pose_index <= 0
            || pose_index >= static_cast<int>(poses.size()) ) {
            AINFO << "params error. poses size:" << poses.size()
                  << ",pose_index:" << pose_index;
            return true;
        }

        unsigned int position_type = poses[pose_index].position_type;
        float diff_age = poses[pose_index].diff_age;
        double local_std = poses[pose_index].local_std;

        if ( _sp_conf->position_type_range.find(position_type)
             != _sp_conf->position_type_range.end()
             && diff_age >= _sp_conf->diff_age_range.first
             && diff_age <= _sp_conf->diff_age_range.second
             && local_std <= _sp_conf->local_std_upper_limit ) {
            return true;
        }
        return false;
    }

    ErrorCode get_return_state() {
        return _return_state;
    }

 protected:
    void update_pose_info(const FramePose& pose,
        std::shared_ptr<BadOrGoodPoseInfo> sp_pose_info) {
        BadOrGoodPoseInfo &pose_info = *sp_pose_info;
        if ( pose_info.pose_count == 0 ) {
            pose_info.start_time = pose.time_stamp;
            pose_info.pose_count++;
            fprintf(stderr, "update start time:%lf,pose count:%d\n",
                pose_info.start_time, pose_info.pose_count);
        } else {
            pose_info.end_time = pose.time_stamp;
            pose_info.pose_count++;
            fprintf(stderr, "update end time:%lf,pose count:%d\n",
                pose_info.end_time, pose_info.pose_count);
        }
    }

    void clear_pose_info(std::shared_ptr<BadOrGoodPoseInfo> sp_pose_info) {
        BadOrGoodPoseInfo &pose_info = *sp_pose_info;
        pose_info.start_time = pose_info.end_time = -1.0;
        pose_info.pose_count = 0;
    }

    int time_to_index(std::vector<FramePose> poses, double time) {
        // int res = -1;
        size_t size = poses.size();
        if ( size == 0 || time <= 0 ) {
            return -1;
        }

        for (size_t i = 0; i < size; i++) {
            if ( poses[i].time_stamp >= time ) {
                return static_cast<int>(i);
            }
        }
        return static_cast<int>(size);
    }


 protected:
    double _progress, _last_progress;
    double _start_time, _end_time;
    int _start_index, _end_index;
    // BadOrGoodPoseInfo _bad_pose_info, _good_pose_info;
    std::shared_ptr<BadOrGoodPoseInfo> _sp_bad_pose_info = nullptr;
    std::shared_ptr<BadOrGoodPoseInfo> _sp_good_pose_info = nullptr;
    ErrorCode _return_state;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_ALIGNMENT_HPP 
