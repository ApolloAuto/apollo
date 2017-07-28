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
#ifndef  MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_HM_OBJECT_TRACK_H_
#define  MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_HM_OBJECT_TRACK_H_
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <queue>
#include "modules/common/macro.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/common/circular_array.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/base_filter.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/tracked_object.h"

namespace apollo {
namespace perception {

class ObjectTrack {
public:
    explicit ObjectTrack(TrackedObjectPtr obj);
    ~ObjectTrack();

    // @brief set filter method for all the object track objects
    // @params[IN] filter_method: method name of filtering algorithm
    // @return nothing
    static void set_filter_method(const std::string& filter_method);
    
    // @brief get next avaiable track id
    // @return next avaiable track id
    static int get_next_track_id();
    
    // @brief predict the state of track
    // @params[IN] time_diff: time interval for predicting
    // @return predicted states of track
    Eigen::VectorXf predict(const double time_diff);   
    
    // @brief update track with object
    // @params[IN] new_object: new object for current updating
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void update_with_object(TrackedObjectPtr& new_object, 
            const double time_diff);

    // @brief update track without object
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void update_without_object(const double time_diff);

    // @brief update track without object with given predicted state
    // @params[IN] predict_state: given predicted state of track
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void update_without_object(const Eigen::VectorXf& predict_state, 
            const double time_diff);

protected:
    /* update with object */
    
    // @brief check whether given measurement is outlier
    // @params[IN] new_object: new detected object for updating
    // @params[IN] time_diff: time interval from last updating
    // @return true if given object is outlier, otherwise return false
    bool check_measurement_outliers(TrackedObjectPtr& new_object, 
            const double time_diff);

    // @brief smooth velocity over track history
    // @params[IN] new_object: new detected object for updating
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void smooth_track_velocity(TrackedObjectPtr& new_object,
            const double time_diff);

    // @brief smooth orientation over track history
    // @return nothing
    void smooth_track_orientation();

    // @brief smooth track class idx over track history
    // @return nothing
    void smooth_track_class_idx();

    // @brief check whether track is static or not
    // @params[IN] new_object: new detected object just updated
    // @params[IN] time_diff: time interval between last two updating
    // @return true if track is static, otherwise return false
    bool check_track_static_hypothesis(ObjectPtr& new_object, 
            const double time_diff);

    // @brief sub strategy of checking whether track is static or not via considering bbox shift
    // @params[IN] new_object: new detected object just updated
    // @params[IN] time_diff: time interval between last two updating
    // @return true if track is static, otherwise return false
    bool check_track_static_hypothesis_by_bbox_shift(ObjectPtr& new_object,
            const double time_diff);

    // @brief sub strategy of checking whether track is static or not via considering the
    // consistancy of direction of velocity
    // @params[IN] new_object: new detected object just updated
    // @params[IN] time_diff: time interval between last two updating
    // @return true if track is static, otherwise return false
    bool check_track_static_hypothesis_by_history_velocity_consistancy(ObjectPtr& new_object,
            const double time_diff);

    // @brief sub strategy of checking whether track is static or not via considering the
    // velocity angle change
    // @params[IN] new_object: new detected object just updated
    // @params[IN] time_diff: time interval between last two updating
    // @return true if track is static, otherwise return false
    bool check_track_static_hypothesis_by_velocity_angle_change(ObjectPtr& new_object,
            const double time_diff);

    // @brief post process track with rule-based tuning or onroad strategies
    // @return nothing
    void post_processing();

private:
    DISALLOW_COPY_AND_ASSIGN(ObjectTrack);
    ObjectTrack();

public:
    // algorithm setup
    static std::string                  _s_filter_method;

    BaseFilter*                     _filter;

    // basic info
    int                                 _idx;
    int                                 _age;
    int                                 _total_visible_count;
    int                                 _consecutive_invisible_count;
    double                              _period;

    TrackedObjectPtr                    _current_object;

    // history
    std::deque<TrackedObjectPtr>        _history_objects;

    // history type info
    std::vector<float>                  _accumulated_type_probs;
    int                                 _type_life_time;

    // states
    // NEED TO NOTICE: All the states would be collected mainly based on states of tracked object.
    // Thus, update tracked object when you update the state of track !!!!!
    bool                                _is_static_hypothesis;
    Eigen::Vector3f                     _belief_anchor_point;
    Eigen::Vector3f                     _belief_velocity;
    Eigen::Vector3f                     _belief_velocity_accelaration;
     
private:
    // global setup
    static int                          _s_track_idx;  
    static const int                    _s_max_cached_object_size = 20;
    static constexpr double             _s_claping_speed_threshold = 0.4;
    static constexpr double             _s_claping_accelaration_threshold = 10;

}; // class ObjectTrack

typedef ObjectTrack* ObjectTrackPtr;

class ObjectTrackSet{
public:    
    ObjectTrackSet(); 
    ~ObjectTrackSet();

    inline std::vector<ObjectTrackPtr>& get_tracks() {
        return _tracks;
    }
    
    inline const std::vector<ObjectTrackPtr>& get_tracks() const {
        return _tracks;
    }
    
    inline int size() const {
        return _tracks.size();
    }
    
    void add_track(const ObjectTrackPtr& track){
        _tracks.push_back(track);
    }
    
    int remove_lost_tracks();

    void clear();
    
private:
    std::vector<ObjectTrackPtr>  _tracks;
    int                          _age_threshold;
    double                       _minimum_visible_ratio;
    int                          _maximum_consecutive_invisible_count;

}; // class ObjectTrackSet

} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_HM_OBJECT_TRACK_H_
