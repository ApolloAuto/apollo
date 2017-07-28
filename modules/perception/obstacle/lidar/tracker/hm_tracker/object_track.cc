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
#include <algorithm>
#include <string>
#include <vector>
#include "modules/common/log.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/kalman_filter.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/object_track.h"
namespace apollo {
namespace perception {

int ObjectTrack::_s_track_idx = 0;
std::string ObjectTrack::_s_filter_method = "direct_kalman_filter";

void ObjectTrack::set_filter_method(const std::string& filter_method) {
    // Set filter method for all the track objects
    if (filter_method == "direct_kalman_filter") {
        _s_filter_method = filter_method;
    } else {
        AINFO << "invalid filter_method!";
    }
    AINFO << "track filter algorithm is " << _s_filter_method;
}

int ObjectTrack::get_next_track_id() {
    // Get next avaiable track id
    int ret_track_id = _s_track_idx;
    if (_s_track_idx == INT_MAX) {
        _s_track_idx = 0;
    } else {
        _s_track_idx++;        
    }
    return ret_track_id;
}

ObjectTrack::ObjectTrack(TrackedObjectPtr obj) {
    // Setup filter
    Eigen::Vector3f initial_anchor_point = obj->anchor_point;
    Eigen::Vector3f initial_velocity = Eigen::Vector3f::Zero();

    _filter = new KalmanFilter();

    _filter->Initialize(initial_anchor_point, initial_velocity);

    // Initialize track info
    _idx = ObjectTrack::get_next_track_id();
    _age = 1;
    _total_visible_count = 1;
    _consecutive_invisible_count = 0;
    _period = 0.0;
    _current_object = obj;
    _accumulated_type_probs = std::vector<float>(MAX_OBJECT_TYPE, 0.0);
    // _accumulated_type_probs[obj->type] += 1.0;
    _accumulated_type_probs = obj->object_ptr->type_probs;
    _type_life_time = 0;

    // Initialize track states
    _is_static_hypothesis = false;
    _belief_anchor_point = initial_anchor_point;
    _belief_velocity = initial_velocity;
    _belief_velocity_accelaration = Eigen::Vector3f::Zero();
    
    // NEED TO NOTICE: All the states would be collected mainly based on states of tracked object.
    // Thus, update tracked object when you update the state of track !!!!!
    obj->velocity = initial_velocity;
}

ObjectTrack::~ObjectTrack() {
    if (_filter) {
        delete _filter;
        _filter = NULL;
    }
}

Eigen::VectorXf ObjectTrack::predict(const double time_diff) {
    // Predict the state of track

    /* Previously, we use the predict of filtering algorithm directly. However, it is hard to
     * ensure that measurements are perfect all the time. To avoid bad estimation generated 
     * from imperfect detection, we use filtering result as prior guidance, while some other
     * post-processing may correct the state of track after filtering. 
     * Thus, aftering predicting of filtering algorithm, we return predicts of states of track.
     * NEED TO NOTICE: predicting is a very necessary part of filtering algorithm !!!!!
     * */

    // Get the predict of filter
    Eigen::VectorXf filter_predict = _filter->Predict(time_diff);

    // Get the predict of track
    Eigen::VectorXf track_predict = filter_predict;

    track_predict(0) = _belief_anchor_point(0) + _belief_velocity(0) * time_diff; 
    track_predict(1) = _belief_anchor_point(1) + _belief_velocity(1) * time_diff;
    track_predict(2) = _belief_anchor_point(2) + _belief_velocity(2) * time_diff;
    track_predict(3) = _belief_velocity(0);
    track_predict(4) = _belief_velocity(1);
    track_predict(5) = _belief_velocity(2);

    return track_predict;
}

void ObjectTrack::update_with_object(TrackedObjectPtr& new_object, 
        const double time_diff) {
    // Update track with object
    if (new_object == nullptr) {
        update_without_object(time_diff);
    }

    /* 1. update object track */
    // 1.1 check measurement outliers
    bool updating_new_object_is_outlier = check_measurement_outliers(new_object, time_diff);
    
    // 1.2 update with object if observation is not outlier
    if (!updating_new_object_is_outlier) {
        _filter->UpdateWithObject(new_object, _current_object, time_diff);
        _filter->GetState(&_belief_anchor_point, &_belief_velocity);
    } else {
        /* Here, we only update belief anchor point with anchor point of new detected object. In 
         * the future, new method that handle outlier could be designed to handle more complicated
         * strategies. 
         * @author zhangye09@baidu.com
         * @date 2017/03/29 */
        _belief_anchor_point = new_object->anchor_point;
    }
 
    // NEED TO NOTICE: All the states would be collected mainly based on states of tracked object.
    // Thus, update tracked object when you update the state of track !!!!!
    new_object->anchor_point = _belief_anchor_point;
    new_object->velocity = _belief_velocity;

    _belief_velocity_accelaration = (new_object->velocity - _current_object->velocity) / time_diff;
    
    /* Currently, we only considered outliers' influence on motion estimation. Track level 
     * smoothness of orientation & class idx may also take into acount it in the future. 
     * @author zhangye09@baidu.com
     * @date 2017/03/29 */
 
    // 1.4 update track info
    _age++;
    _total_visible_count++;
    _consecutive_invisible_count = 0;
    _period += time_diff;

    // 1.5 update history
    if (_history_objects.size() >= _s_max_cached_object_size) {
        _history_objects.pop_front();
    }
    _history_objects.push_back(_current_object);
    _current_object = new_object;

    /* Previously, velocity of object track is smoothed after the smoothing of orientation & type.
     * However, compared to the output of original filter, smoothed velocity, which is a posteriori,
     * is more reasonable to be used in the smoothing of orientation & type.
     * @author zhangye09@baidu.com
     * @date 2017/05/27 */

    /* Previously, track static hypothesis is checked before smoothing strategies. Now, it has been
     * moved in the method of smooth track velocity. This is more reasonable, as track static 
     * hypothesis is decided by velocity more directly when velocity estimation improved. 
     * @author zhangye09@baidu.com
     * @date 2017/05/27/ */

    /* 2. smooth object track */
    // 2.1 smooth velocity
    smooth_track_velocity(new_object, time_diff);

    // 2.2 smooth orientation
    smooth_track_orientation();
    
    // 2.3 smooth class idx
    smooth_track_class_idx();
    
    /* 3. post-processing */
    post_processing();
}

void ObjectTrack::update_without_object(const double time_diff) {
    // Update track's obstacle object
    TrackedObjectPtr new_obj(new TrackedObject());
    new_obj->clone(*_current_object);
    
    Eigen::Vector3f predicted_shift = _belief_velocity * time_diff;
    new_obj->anchor_point = _current_object->anchor_point + predicted_shift;
    new_obj->barycenter = _current_object->barycenter + predicted_shift;
    new_obj->center = _current_object->center + predicted_shift;
    
    // Update obstacle cloud
    pcl_util::PointCloudPtr pc = new_obj->object_ptr->cloud;
    for (size_t j = 0; j < pc->points.size(); ++j) {
        pc->points[j].x += predicted_shift[0];
        pc->points[j].y += predicted_shift[1];
        pc->points[j].z += predicted_shift[2];        
    }

    // Update obstacle polygon
    PolygonDType& polygon = new_obj->object_ptr->polygon;
    for (size_t j = 0; j < polygon.points.size(); ++j) {
        polygon.points[j].x += predicted_shift[0];
        polygon.points[j].y += predicted_shift[1];
        polygon.points[j].z += predicted_shift[2];
    }

    // Update filter
    _filter->UpdateWithoutObject(time_diff);

    // Update states of track
    _belief_anchor_point = new_obj->anchor_point;

    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object included temporaryly occluded ones. Thus, update
    // tracked object when you update the state of track !!!!
    new_obj->velocity = _belief_velocity;

    // Update track info
    _age++;
    _consecutive_invisible_count++;
    _period += time_diff;

    // Update history
    if (_history_objects.size() >= _s_max_cached_object_size) {
        _history_objects.pop_front();
    }
    _history_objects.push_back(_current_object);
    _current_object = new_obj;
}

void ObjectTrack::update_without_object(const Eigen::VectorXf& predict_state, 
        const double time_diff) {
    // Update track's obstacle object
    TrackedObjectPtr new_obj(new TrackedObject());
    new_obj->clone(*_current_object);
    
    Eigen::Vector3f predicted_shift = predict_state.tail(3) * time_diff;
    new_obj->anchor_point = _current_object->anchor_point + predicted_shift;
    new_obj->barycenter = _current_object->barycenter + predicted_shift;
    new_obj->center = _current_object->center + predicted_shift;
    
    // Update obstacle cloud
    pcl_util::PointCloudPtr pc = new_obj->object_ptr->cloud;
    for (size_t j = 0; j < pc->points.size(); ++j) {
        pc->points[j].x += predicted_shift[0];
        pc->points[j].y += predicted_shift[1];
        pc->points[j].z += predicted_shift[2];        
    }

    // Update obstacle polygon
    PolygonDType& polygon = new_obj->object_ptr->polygon;
    for (size_t j = 0; j < polygon.points.size(); ++j) {
        polygon.points[j].x += predicted_shift[0];
        polygon.points[j].y += predicted_shift[1];
        polygon.points[j].z += predicted_shift[2];
    }

    /* No need to update filter*/

    // Update filter
    _filter->UpdateWithoutObject(time_diff);

    // Update states of track
    _belief_anchor_point = new_obj->anchor_point;

    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object included temporaryly occluded ones. Thus, update
    // tracked object when you update the state of track !!!!
    new_obj->velocity = _belief_velocity;

    // Update track info
    _age++;
    _consecutive_invisible_count++;
    _period += time_diff;

    // Update history
    if (_history_objects.size() >= _s_max_cached_object_size) {
        _history_objects.pop_front();
    }
    _history_objects.push_back(_current_object);
    _current_object = new_obj;
}

bool ObjectTrack::check_measurement_outliers(TrackedObjectPtr& new_object, 
        const double time_diff) {
    // Check whether given measurement is outlier
    return false;
}

void ObjectTrack::smooth_track_velocity(TrackedObjectPtr& new_object,
        const double time_diff) {
    // Smooth velocity over track history
    // 1. keep motion if accelaration of filter is greater than a threshold
    Eigen::Vector3f filter_anchor_point = Eigen::Vector3f::Zero();
    Eigen::Vector3f filter_velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f filter_velocity_accelaration = Eigen::Vector3f::Zero();
    _filter->GetState(&filter_anchor_point, &filter_velocity, &filter_velocity_accelaration);
    double filter_accelaration = filter_velocity_accelaration.norm();
    bool need_keep_motion = filter_accelaration >
        _s_claping_accelaration_threshold;
    // use tighter threshold for pedestrian
    if (filter_accelaration > _s_claping_accelaration_threshold / 2 && 
        _current_object->object_ptr->type == PEDESTRIAN) {
        need_keep_motion = true;
    }
    if (need_keep_motion) {
        Eigen::Vector3f last_velocity = Eigen::Vector3f::Zero();
        if (_history_objects.size() > 0) {
            last_velocity = 
                _history_objects[_history_objects.size() - 1]->velocity;
        }
        _belief_velocity = last_velocity;
        _belief_velocity_accelaration = Eigen::Vector3f::Zero();

        // NEED TO NOTICE: All the states would be collected mainly based on
        // states of tracked object included temporaryly occluded ones. Thus,
        // update tracked object when you update the state of track !!!!
        _current_object->velocity = _belief_velocity;
 
        // keep static hypothesis
        return;
    }
    // 2. static hypothesis check
    _is_static_hypothesis = check_track_static_hypothesis(
            new_object->object_ptr, time_diff);

    if (_is_static_hypothesis) {
        _belief_velocity = Eigen::Vector3f::Zero();
        _belief_velocity_accelaration = Eigen::Vector3f::Zero();

        // NEED TO NOTICE: All the states would be collected mainly based on
        // states of tracked object included temporaryly occluded ones. Thus,
        // update tracked object when you update the state of track !!!!
        _current_object->velocity = _belief_velocity;
    }
}

void ObjectTrack::smooth_track_orientation() {
    // Smooth orientation over track history
    float cur_speed = _current_object->velocity.head(2).norm();
    Eigen::Vector3f cur_obj_dir = _current_object->direction;
    cur_obj_dir.normalize();
    Eigen::Vector3f cur_motion_dir = _current_object->velocity;
    cur_motion_dir.normalize();

    TrackedObjectPtr previous_obj = _history_objects.back();
    float previous_speed = previous_obj->velocity.norm();
    Eigen::Vector3f previous_motion_dir = previous_obj->velocity;
    previous_motion_dir.normalize();
    Eigen::Vector3f previous_dir = previous_obj->direction;

    double acceleration = (_current_object->velocity -
            previous_obj->velocity).head(2).norm();
    double w_vel = pow(10, std::min(12.0f, cur_speed - 1));

    double cos_val = previous_motion_dir.dot(cur_motion_dir);
    double motion_confidence = 1;

    if (_age > 1 && acceleration > 50) {
        motion_confidence = 0.01;
        w_vel = motion_confidence;
    }

    if (cur_speed > 1.0f) {
        float dot_val_00 = cur_obj_dir[0] * cur_motion_dir[0] +
            cur_obj_dir[1] * cur_motion_dir[1];
        float dot_val_01 = cur_obj_dir[0] * cur_motion_dir[1] -
            cur_obj_dir[1] * cur_motion_dir[0];
        if (fabs(dot_val_00) >= fabs(dot_val_01)) {
            if (dot_val_00 < 0) {
                cur_obj_dir = -cur_obj_dir;
            }
        } else {
            if (dot_val_01 < 0) {
                cur_obj_dir = Eigen::Vector3f(
                        cur_obj_dir[1], -cur_obj_dir[0], 0);
            } else {
                cur_obj_dir = Eigen::Vector3f(
                        -cur_obj_dir[1], cur_obj_dir[0], 0);
            }
        }
    }

    Eigen::Vector3f obs_dir = cur_obj_dir + cur_motion_dir * w_vel;
    obs_dir.normalize();
    if (cur_speed > 2.0f) {
        float dot_val_00 = obs_dir[0] * previous_dir[0] +
            obs_dir[1] * previous_dir[1];
        float dot_val_01 = obs_dir[0] * previous_dir[1] -
            obs_dir[1] * previous_dir[0];
        if (fabs(dot_val_00) >= fabs(dot_val_01)) {
            if (dot_val_00 < 0) {
                previous_dir = -previous_dir;
            }
        } else {
            if (dot_val_01 > 0) {
                previous_dir = Eigen::Vector3f(
                        previous_dir[1], -previous_dir[0], 0);
            } else {
                previous_dir = Eigen::Vector3f(
                        -previous_dir[1], previous_dir[0], 0);
            }
        }
    } else {
        float dot_val_00 = obs_dir[0] * previous_dir[0] +
            obs_dir[1] * previous_dir[1];
        float dot_val_01 = obs_dir[0] * previous_dir[1] -
            obs_dir[1] * previous_dir[0];
        if (fabs(dot_val_00) >= fabs(dot_val_01)) {
            if (dot_val_00 < 0) {
                obs_dir = -obs_dir;
            }
        } else {
            if (dot_val_01 > 0) {
                obs_dir = Eigen::Vector3f(obs_dir[1], -obs_dir[0], 0);
            } else {
                obs_dir = Eigen::Vector3f(-obs_dir[1], obs_dir[0], 0);
            }
        }
    }

    Eigen::Vector3f result_dir = previous_dir + obs_dir * w_vel;
    if (result_dir.norm() < FLT_EPSILON) {
        result_dir = previous_dir;
    }
    result_dir.normalize();

    Eigen::Vector3d bk_size = _current_object->size.cast<double>();
    Eigen::Vector3d bk_dir = _current_object->direction.cast<double>();
    Eigen::Vector3d bk_center = _current_object->center.cast<double>();

    Eigen::Vector3d new_size;
    Eigen::Vector3d new_center;
    compute_bbox_size_center_xy<pcl_util::Point>(
            _current_object->object_ptr->cloud,
            result_dir.cast<double>(), new_size, new_center);

    if (new_size[0] * new_size[1] < 1.2 * bk_size[0] * bk_size[1]) {
        _current_object->direction = result_dir;
        _current_object->center = new_center.cast<float>();
        _current_object->size = new_size.cast<float>();
    }
}

void ObjectTrack::smooth_track_class_idx() {
}

bool ObjectTrack::check_track_static_hypothesis(ObjectPtr& new_object,
        const double time_diff) {
    // Check whether track is static
    bool is_velocity_angle_change =
        check_track_static_hypothesis_by_velocity_angle_change(new_object,
                time_diff);
    // Define track as static & clap velocity to 0
    // when velocity angle change & it is smaller than a threshold
    double speed = _belief_velocity.head(2).norm();
    bool velocity_is_small = speed < (_s_claping_speed_threshold / 2);
    // use loose threshold for pedestrian
    if (speed < _s_claping_speed_threshold &&
        _current_object->object_ptr->type != PEDESTRIAN) {
        velocity_is_small = true;
    }
    // Need to notice: claping small velocity may not reasonable when the true
    // velocity of target object is really small. e.g. a moving out vehicle in
    // a parking lot. Thus, instead of clapping all the small velocity, we clap
    // those whose history trajectory or performance is close to a static one.
    if (velocity_is_small && is_velocity_angle_change) {
        return true;
    }
    return false;
}

bool ObjectTrack::check_track_static_hypothesis_by_bbox_shift(
        ObjectPtr& new_object,
        const double time_diff) {
    // Sub strategy of checking whether track is static or not - let track be
    // static if bbox shift within a small threshold in last two frames
    TrackedObjectPtr pre_object = _current_object;
    Eigen::Vector3f pre_c = pre_object->center;
    Eigen::Vector3f pre_dir = pre_object->direction;
    Eigen::Vector3f pre_size = pre_object->size;
    Eigen::Vector3f cur_c = new_object->center.cast<float>();
    Eigen::Vector3f cur_dir = new_object->direction.cast<float>();
    Eigen::Vector3f cur_size(
            new_object->length, new_object->width, new_object->height);

    float dist_p2c = dist_xy_to_bbox(cur_c, cur_dir, cur_size, pre_c);
    float dist_c2p = dist_xy_to_bbox(pre_c, pre_dir, pre_size, cur_c);
    float contain_dist_threshold = 0.2f;
    if (dist_c2p > contain_dist_threshold && dist_p2c >
            contain_dist_threshold) {
        return false;
    }

    float bbox_max_dist_threshold = 1 * time_diff;
    float bbox_min_dist_threshold = 0.4 * time_diff;
    float bb_min_dist = 0;
    float bb_max_dist = 0;
    if (dist_p2c <= contain_dist_threshold) {
        max_min_distance_xy_bbox_to_bbox(pre_c, pre_dir, pre_size,
            cur_c, cur_dir, cur_size, bb_max_dist, bb_min_dist);
        if (bb_min_dist < bbox_min_dist_threshold && bb_max_dist <
                bbox_max_dist_threshold) {
            return true;
        }
    }

    if (dist_c2p <= contain_dist_threshold) {
        max_min_distance_xy_bbox_to_bbox(cur_c, cur_dir, cur_size,
            pre_c, pre_dir, pre_size, bb_max_dist, bb_min_dist);
        if (bb_min_dist < bbox_min_dist_threshold && bb_max_dist <
                bbox_max_dist_threshold) {
            return true;
        }
    }

    return false;
}

bool ObjectTrack::check_track_static_hypothesis_by_history_velocity_consistancy(
        ObjectPtr& new_object,
        const double time_diff) {
    // Sub strategy of checking whether track is static or not - let track be
    // static if consistancy of heading velocity is larger than a treshold
    std::vector<Eigen::Vector3f> history_centers;
    for (int i = 0; i < _history_objects.size(); ++i) {
        history_centers.push_back(_history_objects[i]->anchor_point);
    }
    int center_num = history_centers.size();
    if (center_num <= 5) {
        return false;
    }
    int observed_frame_num = 20;
    if (observed_frame_num > center_num) {
        observed_frame_num = center_num;
    }
    Eigen::MatrixXf center_matrix(observed_frame_num, 2);
    for (int i = 0; i < center_matrix.rows(); ++i) {
        center_matrix.row(i) = history_centers[center_num - 1 - i].head(2);
    }

    Eigen::MatrixXf centered = center_matrix.rowwise() -
        center_matrix.colwise().mean();
    Eigen::MatrixXf cov = (centered.adjoint() * centered) /
        (center_matrix.rows() - 1);
    Eigen::EigenSolver<Eigen::MatrixXf> eigen_solver;
    eigen_solver.compute(cov, false);
    Eigen::MatrixXf eigen_values = eigen_solver.pseudoEigenvalueMatrix();
    if (std::max(eigen_values(0, 0), eigen_values(1, 1)) < 2.0) {
        return true;
    }

    return false;
}

bool ObjectTrack::check_track_static_hypothesis_by_velocity_angle_change(
        ObjectPtr& new_object,
        const double time_diff) {
    // Sub strategy of checking whether track is static or not - let track be
    // static if velocity angle change from previous and current frame is
    // larger than M_PI / 3.0
    Eigen::Vector3f previous_velocity =
        _history_objects[_history_objects.size() - 1]->velocity;
    Eigen::Vector3f current_velocity = _current_object->velocity;
    double velocity_angle_change = vector_theta_2d_xy(
            previous_velocity, current_velocity);
    // Previously, this threshold is set to PI/3. Now, as the smoothness of the
    // filter is improved by Robust Adaptive Kalman Filter, we would use more
    // strict threshold in the future.
    if (fabs(velocity_angle_change) > M_PI / 4.0) {
        return true;
    }
    return false;
}

void ObjectTrack::post_processing() {
}

/*class ObjectTrackSet*/
ObjectTrackSet::ObjectTrackSet(): _age_threshold(5),
    _minimum_visible_ratio(0.6f),
    _maximum_consecutive_invisible_count(1) {
    _tracks.reserve(1000);
}

ObjectTrackSet::~ObjectTrackSet() {
    std::cout << "Release ObjectTrackSet...\n";
    clear();
    std::cout << "Release ObjectTrackSet End\n";
}

void ObjectTrackSet::clear() {
    for (size_t i = 0; i < _tracks.size(); i++) {
        if (_tracks[i]) {
            delete (_tracks[i]);
            _tracks[i] = NULL;
        }
    }
    _tracks.clear();
}

int ObjectTrackSet::remove_lost_tracks() {
    int track_num = 0;
    for (size_t i = 0; i < _tracks.size(); i++) {
        if (_tracks[i]->_age < _age_threshold &&
            _tracks[i]->_consecutive_invisible_count > 1) {
            continue;
        }

        float visible_ratio = _tracks[i]->_total_visible_count *
            1.0f / _tracks[i]->_age;
        if (visible_ratio < _minimum_visible_ratio) {
            continue;
        }

        if (_tracks[i]->_consecutive_invisible_count
            > _maximum_consecutive_invisible_count) {
            continue;
        }

        if (i == track_num) {
            track_num++;
            continue;
        } else {
            ObjectTrackPtr tmp = _tracks[i];
            _tracks[i] = _tracks[track_num];
            _tracks[track_num] = tmp;
            track_num++;
        }
    }

    int no_removed = _tracks.size() - track_num;
    for (size_t i = track_num; i < _tracks.size(); i++) {
        if (_tracks[i] != NULL) {
            delete (_tracks[i]);
            _tracks[i] = NULL;
        }
    }
    _tracks.resize(track_num);
    return no_removed;
}

}  // namespace perception
}  // namespace apollo
