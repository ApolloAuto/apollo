/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "mix_camera_tracker.h"

namespace adu {
namespace perception {
namespace obstacle {

bool MixCameraTracker::init() {

    // TODO Config here for turing trackers on or off

    bool init_flag = true;

    init_flag &= _cs2d_tracker.init();

    if (_with_dl_feature) {
        init_flag &= _dlf_tracker.init();
    }

//    // TODO Disable DAT for now
//    init_flag &= _dat_tracker.init();

    init_flag &= _kcf_tracker.init();

    return init_flag;
}

bool MixCameraTracker::associate(const cv::Mat &frame,
                                 const std::vector<VisualObjectPtr> &objects,
                                 double timestamp,
                                 const CameraTrackerOptions &options,
                                 std::vector<VisualObjectPtr> *tracked_objects) {
    _frame_cnt++;

    // Profiling
    Timer timer;

    *tracked_objects = objects;  // TODO: What kind of API design is this? Who defined this
    std::map<int, int> id_mapping;  // detect to track

    float scale = 1.0f; // TODO put scale adjustment in config
    cv::Mat img;
    cv::resize(frame, img, cv::Size(static_cast<int>(frame.cols * scale),
                                    static_cast<int>(frame.rows * scale)));
    std::vector<Detected> detected;
    get_detected_from_vo(frame.size(), objects, scale, detected);
    for (auto& dets: detected) {
        dets._last_seen_frame_cnt = _frame_cnt;
        dets._last_seen_timestamp = timestamp;
    }

    if (_verbose) {
        XLOG(INFO) << "#T:" << _tracked.size()
                   << " #D:" << detected.size();
    }

    // (From all trackers)
    // 1. Get affinity matrix
    std::vector<std::vector<float > > affinity_matrix;
    affinity_matrix = std::vector<std::vector<float>>(_tracked.size(),
                                                      std::vector<float>(detected.size(), 1.0f));

    // (0) cs2d
    timer.tic();
    std::vector<std::vector<float > > cs2d_affinity_matrix;
    _cs2d_tracker.set_full_selection(_tracked.size(), detected.size());
    _cs2d_tracker.get_affinity_matrix(img, _tracked, detected, cs2d_affinity_matrix);
    if (_verbose) {
        XLOG(INFO) << "CS2D Affinity time: " << timer.toc() << "ms";
    }

    // Merge
    merge_affinity_matrix(cs2d_affinity_matrix, affinity_matrix);

    // (1) dlf
    if (_with_dl_feature) {
        timer.tic();
        std::vector<std::vector<float > > dlf_affinity_matrix;
        _dlf_tracker.set_full_selection(_tracked.size(), detected.size());
        _dlf_tracker.get_affinity_matrix(img, _tracked, detected, dlf_affinity_matrix);

        // Merge
        merge_affinity_matrix(dlf_affinity_matrix, affinity_matrix);

        // High confidence selection filtering. Thresholds are tuned
        // 1st pass high value as 0.95. No competing high value in the cross
        filter_affinity_matrix(0.95f, 0.95f, affinity_matrix);

        // 2nd pass value above 0.7. The others for 0.3
        filter_affinity_matrix(0.7f, 0.3f, affinity_matrix);

        // 3rd pass value above 0.5. The others for 0.2
        filter_affinity_matrix(0.5f, 0.2f, affinity_matrix);

        if (_verbose) {
            XLOG(INFO) << "DLF Affinity time: " << timer.toc() << "ms";
        }
    }

//    // TODO Disable DAT for now. Upgrade "update_tracked" logic in DAT
//    // (2) dat
//    timer.tic();
//    std::vector<std::vector<float > > dat_affinity_matrix;
//    _dat_tracker.set_selected_entries(affinity_matrix);
//    _dat_tracker.get_affinity_matrix(img, _tracked, detected, dat_affinity_matrix);
//    if (_verbose) {
//        XLOG(INFO) << "DAT Affinity time: " << timer.toc() << "ms";
//    }
//
//    // Merge
//    merge_affinity_matrix(dat_affinity_matrix, affinity_matrix);

    // (3) kcf
    timer.tic();
    std::vector<std::vector<float > > kcf_affinity_matrix;
    _kcf_tracker.set_selected_entries(affinity_matrix);
    _kcf_tracker.get_affinity_matrix(img, _tracked, detected, kcf_affinity_matrix);
    if (_verbose) {
        XLOG(INFO) << "KCF Affinity time: " << timer.toc() << "ms";
    }

    // Merge
    merge_affinity_matrix(kcf_affinity_matrix, affinity_matrix);

    // 2. Matching
    std::unordered_map<int, int> local_matching;
    std::unordered_set<int> local_matched_detected;
    matrix_matching(affinity_matrix, local_matching, local_matched_detected);

    // 3. Tracker and ID management
    tracker_and_id_management(local_matching, local_matched_detected, detected,
                              _tracked, _next_tracked_id, id_mapping, _frame_cnt);

    // (From all trackers)
    // 4. Update information used in tracks for the next frame
    timer.tic();
    _cs2d_tracker.update_tracked(img, detected, _tracked);
    if (_verbose) {
        XLOG(INFO) << "CS2D Update time: " << timer.toc() << "ms";
    }

    if (_with_dl_feature) {
        timer.tic();
        _dlf_tracker.update_tracked(img, detected, _tracked);
        if (_verbose) {
            XLOG(INFO) << "DLF Update time: " << timer.toc() << "ms";
        }
    }

//    // TODO Disable DAT for now
//    timer.tic();
//    _dat_tracker.update_tracked(img, detected, _tracked);
//    if (_verbose) {
//        XLOG(INFO) << "DAT Update time: " << timer.toc() << "ms";
//    }

    timer.tic();
    _kcf_tracker.update_tracked(img, detected, _tracked);
    if (_verbose) {
        XLOG(INFO) << "KCF Update time: " << timer.toc() << "ms";
    }

    // Output: Id mapping from detected id to tracked id
    for (auto obj_ptr: *tracked_objects) {

        obj_ptr->latest_tracked_time = timestamp;

        if (id_mapping.find(obj_ptr->id) != id_mapping.end()) {
            obj_ptr->track_id = id_mapping[obj_ptr->id];
        }
        else {
            // Should not happen since every detected object will have a tracked ID
            XLOG(WARN) << "WARN: Detection" << obj_ptr->id << " has no tracking ID";
            obj_ptr->track_id = -1;
        }
    }

    return true;
}

bool MixCameraTracker::predict_shape(const cv::Mat &frame,
                                          const std::vector<VisualObjectPtr> &objects,
                                          double timestamp,
                                          const CameraTrackerOptions &options,
                                          std::vector<VisualObjectPtr> *tracked_objects) {
    return true;
}

bool MixCameraTracker::predict_velocity(const cv::Mat &frame,
                                             const std::vector<VisualObjectPtr> &objects,
                                             double timestamp,
                                             const CameraTrackerOptions &options,
                                             std::vector<VisualObjectPtr> *tracked_objects) {

    *tracked_objects = objects;

    // update lost_frame_count
    for (auto iter: _tracked_filters) {
        iter.second.lost_frame_count += 1;
    }

    // create and reduce lost_frame_count to 0 if matched. predict here
    for (auto obj_ptr: *tracked_objects) {

        int tracked_id = obj_ptr->track_id;
        double now_timestamp = obj_ptr->latest_tracked_time;

        if (tracked_id != -1) {
            if (_tracked_filters.find(tracked_id) != _tracked_filters.end()) {

                double last_timestamp = _tracked_filters[tracked_id].last_seen_timestamp;

                _tracked_filters[tracked_id].lost_frame_count = 0;
                _tracked_filters[tracked_id].last_seen_timestamp = obj_ptr->latest_tracked_time;

                _tracked_filters[tracked_id].position.predict(now_timestamp - last_timestamp);
                _tracked_filters[tracked_id].orientation.predict(now_timestamp - last_timestamp);
                _tracked_filters[tracked_id].size_3d_1.predict(now_timestamp - last_timestamp);
                _tracked_filters[tracked_id].size_3d_2.predict(now_timestamp - last_timestamp);
            }
            else {
                _tracked_filters[tracked_id] = Filter();
                _tracked_filters[tracked_id].track_id = tracked_id;
                _tracked_filters[tracked_id].last_seen_timestamp = obj_ptr->latest_tracked_time;
            }
        }
    }

    // destroy old filters
    std::vector<int> id_to_destroy;
    for (const auto iter: _tracked_filters) {
        if (iter.second.lost_frame_count > _max_kep_frame_cnt) {
            id_to_destroy.push_back(iter.first);
        }
    }
    for (const auto id: id_to_destroy) {
        _tracked_filters.erase(id);
    }

    // filter orientation and 3d size for tracked_objects in camera space
    for (auto obj_ptr: *tracked_objects) {
        int tracked_id = obj_ptr->track_id;

        if (tracked_id != -1 && _tracked_filters.find(tracked_id) != _tracked_filters.end()) {

            // Regenerate theta...
            // This definition is correct for OpenGL viewer...
            double beta = std::atan2(obj_ptr->center.x(), obj_ptr->center.z());
            double theta = obj_ptr->alpha + beta;
            if (theta > M_PI) {
                theta -= 2*M_PI;
            }
            else if (theta < -M_PI) {
                theta += 2*M_PI;
            }
            obj_ptr->theta = theta;

            // Orientation
            Eigen::Vector2d z;
            z << obj_ptr->alpha, obj_ptr->theta;
            _tracked_filters[tracked_id].orientation.correct(z);
            Eigen::Vector4d x = _tracked_filters[tracked_id].orientation.get_state();
            obj_ptr->alpha = x(0);
            obj_ptr->theta = x(1);

            // 3D size 1
            z << obj_ptr->height, obj_ptr->width;
            _tracked_filters[tracked_id].size_3d_1.correct(z);
            x = _tracked_filters[tracked_id].size_3d_1.get_state();
            obj_ptr->height = x(0);
            obj_ptr->width = x(1);

            // 3D size 2
            z << obj_ptr->length, 0.0;
            _tracked_filters[tracked_id].size_3d_2.correct(z);
            x = _tracked_filters[tracked_id].size_3d_2.get_state();
            obj_ptr->length = x(0);
        }
    }

    // do orientation filter before this, cause it update everything about orientation and update the original
    this->trans_object_to_world(options, tracked_objects);

    // filter position for tracked_objects in world space
    for (auto obj_ptr: *tracked_objects) {
        int tracked_id = obj_ptr->track_id;

        if (tracked_id != -1 && _tracked_filters.find(tracked_id) != _tracked_filters.end()) {
            Eigen::Vector2d z;
            z << obj_ptr->center(0), obj_ptr->center(1);
            _tracked_filters[tracked_id].position.correct(z);

            Eigen::Vector4d x = _tracked_filters[tracked_id].position.get_state();
            obj_ptr->center(0) = x[0];
            obj_ptr->center(1) = x[1];
            obj_ptr->velocity(0) = x[2];
            obj_ptr->velocity(1) = x[3];
            obj_ptr->velocity(2) = 0;
        }
    }

    return true;
}

std::string MixCameraTracker::name() const {
    return "MixCameraTracker";
}

REGISTER_CAMERA_TRACKER(MixCameraTracker);

}  // namespace obstacle
}  // namespace perception
}  // namespace adu
