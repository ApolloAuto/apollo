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
 
#ifndef ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_H
#define ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_H

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_motion_fusion.h"

namespace apollo {
namespace perception {

class PbfTrack {
public:
    explicit PbfTrack(PbfSensorObjectPtr obj);

    ~PbfTrack();

    /**@brief Update track with sensor object */
    void update_with_sensor_object(PbfSensorObjectPtr obj, double match_dist);

    void update_without_sensor_object(const SensorType& sensor_type,
        const std::string& sensor_id, double min_match_dist, double timestamp);

    PbfSensorObjectPtr get_fused_object();

    double get_fused_timestamp() const;

    PbfSensorObjectPtr get_lidar_object(const std::string& sensor_id);

    PbfSensorObjectPtr get_radar_object(const std::string& sensor_id);

    PbfSensorObjectPtr get_sensor_object(const SensorType& sensor_type, 
        const std::string& sensor_id);
    
    /**@brief get latest lidar measurement for multi lidar sensors*/
    PbfSensorObjectPtr get_latest_lidar_object();
    /**@brief get latest lidar measurement for multi radar sensors*/    
    PbfSensorObjectPtr get_latest_radar_object();

    int get_track_id() const;

    double get_tracking_period() const {
        return _tracking_period;
    }

    inline bool is_dead() const {
        return _is_dead;
    }
    bool able_to_publish();

    static int get_next_track_id();

    static void set_max_lidar_invisible_period(double period) {
        _s_max_lidar_invisible_period = period;
    }

    static double get_max_lidar_invisible_period() {
        return _s_max_lidar_invisible_period;
    }

    static void set_max_radar_invisible_period(double period) {
        _s_max_radar_invisible_period = period;
    }

    static double get_max_radar_invisible_period() {
        return _s_max_radar_confident_angle;
    }

    static void set_max_radar_confident_angle(double angle) {
        _s_max_radar_confident_angle = angle;
    }

    static double get_max_radar_confident_angle() {
        return _s_max_radar_confident_angle;
    }

    static void set_min_radar_confident_distance(double dist) {
        _s_min_radar_confident_distance = dist;
    }

    static void set_publish_if_has_lidar(bool enabled) {
        _s_publish_if_has_lidar = enabled;
    }
    static void set_publish_if_has_radar(bool enabled) {
        _s_publish_if_has_radar = enabled;
    }

    static void set_motion_fusion_method(const std::string motion_fusion_method);

protected:
    /**@brief use obj's velocity to update obj's location to input timestamp*/
    void perform_motion_compensation(PbfSensorObjectPtr obj, double timestamp);

    void perform_motion_fusion(PbfSensorObjectPtr obj);

    void update_measurements_life_with_measurement(std::map<std::string, 
        PbfSensorObjectPtr>& objects, const std::string& sensor_id, 
        double timestamp, double max_invisible_time);
    
    void update_measurements_life_without_measurement(std::map<std::string, 
        PbfSensorObjectPtr>& objects, const std::string& sensor_id, 
        double timestamp, double max_invisible_time, bool& invisible_state);

protected:
    PbfSensorObjectPtr    _fused_object;

    /**@brief time stamp of the track*/
    double                _fused_timestamp;

    int                   _age;
    double                _tracking_period;

    /**@brief global track id*/
    int                   _idx;
    double                _invisible_period;
    bool                  _invisible_in_lidar;
    bool                  _invisible_in_radar;

    /**@brief motion fusion*/
    PbfBaseMotionFusion*  _motion_fusion;

    /**@brief one object instance per sensor, might be more later*/
    std::map<std::string, PbfSensorObjectPtr>  _lidar_objects;
    std::map<std::string, PbfSensorObjectPtr>  _radar_objects;

    bool                  _is_dead;
private:
    PbfTrack();

private:
    static int            _s_track_idx;

    //invisible period for different sensors
    static double         _s_max_lidar_invisible_period;
    static double         _s_max_radar_invisible_period;

    //radar confidant regions
    static double         _s_max_radar_confident_angle;
    static double         _s_min_radar_confident_distance;

    static std::string    _s_motion_fusion_method;

    //publish conditions
    static bool           _s_publish_if_has_lidar;
    static bool           _s_publish_if_has_radar;
};

typedef std::shared_ptr<PbfTrack> PbfTrackPtr;

} // namespace perception
} // namespace apollo

#endif
