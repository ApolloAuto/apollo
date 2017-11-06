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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

#include "boost/format.hpp"

#include "modules/perception/obstacle/base/types.h"
#include "modules/common/macro.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"

namespace apollo {
namespace perception {
/*class PbfTrack*/
int PbfTrack::_s_track_idx = 0;
double PbfTrack::_s_max_lidar_invisible_period = 0.25;
double PbfTrack::_s_max_radar_invisible_period = 0.15;
double PbfTrack::_s_max_radar_confident_angle = 20;
double PbfTrack::_s_min_radar_confident_distance = 40;
std::string PbfTrack::_s_motion_fusion_method = "PbfKalmanMotionFusion";

bool PbfTrack::_s_publish_if_has_lidar = true;
bool PbfTrack::_s_publish_if_has_radar = true;

PbfTrack::PbfTrack(PbfSensorObjectPtr obj){
    _idx = get_next_track_id();
    SensorType sensor_type = obj->sensor_type;
    std::string sensor_id = obj->sensor_id;
    _invisible_in_lidar = true;
    _invisible_in_radar = true;

    if (_s_motion_fusion_method == "PbfKalmanMotionFusion") {
        _motion_fusion = new PbfKalmanMotionFusion();
    } else {
        _motion_fusion = new PbfKalmanMotionFusion();
    }

    if (is_lidar(sensor_type)) {
        _lidar_objects[sensor_id] = obj;
        _motion_fusion->initialize(obj);
        _invisible_in_lidar = false;
    } else if (is_radar(sensor_type)) {
        _radar_objects[sensor_id] = obj;
        _motion_fusion->initialize(obj);
        _invisible_in_radar = false;
    } else {
        AERROR << "Unsupported sensor type : "
            << sensor_type << ", sensor id : " << sensor_id;
    }

    _fused_timestamp = obj->timestamp;
    _fused_object.reset(new PbfSensorObject());
    _fused_object->clone(*obj);
    _age = 0;
    _invisible_period = 0;
    _tracking_period = 0.0;
    _is_dead = false;
}

void PbfTrack::set_motion_fusion_method(const std::string motion_fusion_method) {
    if (motion_fusion_method == "PbfKalmanMotionFusion" ||
        motion_fusion_method == "PbfDirectMotionFusion") {
        _s_motion_fusion_method = motion_fusion_method;
    } else {
        AERROR << "unsupported motion fusion method : " << motion_fusion_method
            << ", use default method : " << _s_motion_fusion_method;
    }
}

PbfTrack::~PbfTrack() {
    if (_motion_fusion != nullptr) {
        delete _motion_fusion;
        _motion_fusion = nullptr;
    }
}

void PbfTrack::update_with_sensor_object(PbfSensorObjectPtr obj, double match_dist) {

    const SensorType& sensor_type = obj->sensor_type;
    const std::string sensor_id = obj->sensor_id;
    perform_motion_fusion(obj);

    if (is_lidar(sensor_type)) {
        _lidar_objects[sensor_id] = obj;
        _invisible_in_lidar = false;
    } else if (is_radar(sensor_type)) {
        _radar_objects[sensor_id] = obj;
        _invisible_in_radar = false;
    }

    double timestamp = obj->timestamp;
    update_measurements_life_with_measurement(_lidar_objects, sensor_id,
        timestamp, _s_max_lidar_invisible_period);
    update_measurements_life_with_measurement(_radar_objects, sensor_id,
        timestamp, _s_max_radar_invisible_period);

    _invisible_period = 0;
    _tracking_period += obj->timestamp - _fused_timestamp;
    _fused_timestamp = obj->timestamp;
    _fused_object->timestamp = obj->timestamp;
}
void PbfTrack::update_without_sensor_object(const SensorType& sensor_type,
    const std::string& sensor_id, double min_match_dist, double timestamp) {
    bool is_alive = false;
    update_measurements_life_without_measurement(_lidar_objects,
        sensor_id, timestamp, _s_max_lidar_invisible_period, _invisible_in_lidar);
    update_measurements_life_without_measurement(_radar_objects,
        sensor_id, timestamp, _s_max_radar_invisible_period, _invisible_in_radar);

    is_alive = (!_lidar_objects.empty() || !_radar_objects.empty());
    _is_dead = !is_alive;

    if (is_alive) {
        double time_diff = timestamp - _fused_timestamp;
        _motion_fusion->update_without_object(time_diff);
        perform_motion_compensation(_fused_object, timestamp);
        _invisible_period = timestamp - _fused_timestamp;
    }
}

int PbfTrack::get_track_id() const {
    return _idx;
}

PbfSensorObjectPtr PbfTrack::get_fused_object() {
    return _fused_object;
}

double PbfTrack::get_fused_timestamp() const {
    return _fused_timestamp;
}

PbfSensorObjectPtr PbfTrack::get_lidar_object(const std::string& sensor_id) {
    PbfSensorObjectPtr obj = nullptr;
    auto it = _lidar_objects.find(sensor_id);
    if (it != _lidar_objects.end()) {
        obj = it->second;
    }
    return obj;
}

PbfSensorObjectPtr PbfTrack::get_radar_object(const std::string& sensor_id) {
    PbfSensorObjectPtr obj = nullptr;
    auto it = _radar_objects.find(sensor_id);
    if (it != _radar_objects.end()) {
        obj = it->second;
    }
    return obj;
}

void PbfTrack::perform_motion_fusion(PbfSensorObjectPtr obj) {

    const SensorType& sensor_type = obj->sensor_type;
    double time_diff = obj->timestamp - _fused_object->timestamp;

    PbfSensorObjectPtr lidar_object = get_latest_lidar_object();
    PbfSensorObjectPtr radar_object = get_latest_radar_object();

    if (is_lidar(sensor_type)) {
        _fused_object->clone(*obj);
        if (_motion_fusion->initialized() &&
            (lidar_object != nullptr || radar_object != nullptr)) {
            _motion_fusion->update_with_object(obj, time_diff);
            Eigen::Vector3d anchor_point;
            Eigen::Vector3d velocity;
            _motion_fusion->get_state(anchor_point, velocity);
            _fused_object->object->velocity = velocity;
        } else {
            //in case the track is created with a camera measurement
            _motion_fusion->initialize(obj);
        }
    } else if (is_radar(sensor_type)) {
        if (_motion_fusion->initialized() &&
            (lidar_object != nullptr || radar_object != nullptr)) {
            Eigen::Vector3d pre_anchor_point;
            Eigen::Vector3d pre_velocity;
            _motion_fusion->get_state(pre_anchor_point, pre_velocity);
            _motion_fusion->update_with_object(obj, time_diff);
            Eigen::Vector3d anchor_point;
            Eigen::Vector3d velocity;
            _motion_fusion->get_state(anchor_point, velocity);
            if (lidar_object == nullptr) {
                PbfSensorObject fused_obj_bk;
                fused_obj_bk.clone(*_fused_object);
                _fused_object->clone(*obj);
                _fused_object->object->velocity = velocity;
            } else {
                //if has lidar, use lidar shape
                Eigen::Vector3d translation = anchor_point - pre_anchor_point;
                _fused_object->object->anchor_point = anchor_point;
                _fused_object->object->center += translation;
                for (auto point : _fused_object->object->polygon.points) {
                    point.x += translation[0];
                    point.y += translation[1];
                    point.z += translation[2];
                }
                _fused_object->object->velocity = velocity;
            }
        } else {
            AERROR << "Something must be wrong.";
        }
    } else {
        AERROR << "Unsupport sensor type " << sensor_type;
        return;
    }

}

void PbfTrack::perform_motion_compensation(PbfSensorObjectPtr obj, double timestamp) {
    double time_diff = timestamp - obj->timestamp;
    if (time_diff < 0) {
        AERROR << "target timestamp is smaller than previous timestamp";
        return;
    }

    Eigen::Vector3d offset = obj->object->velocity * time_diff;
    obj->object->center += offset;
    obj->object->anchor_point += offset;

    PolygonDType& polygon = obj->object->polygon;
    for (int i = 0; i < (int)polygon.size(); i++) {
        polygon.points[i].x += offset[0];
        polygon.points[i].y += offset[1];
        polygon.points[i].z += offset[2];
    }

    pcl_util::PointCloudPtr cloud = obj->object->cloud;
    for (int i = 0; i < (int)cloud->size(); i++) {
        cloud->points[i].x += offset[0];
        cloud->points[i].y += offset[1];
        cloud->points[i].z += offset[2];
    }

    obj->timestamp = timestamp;
    obj->object->latest_tracked_time = timestamp;
    obj->object->tracking_time += time_diff;
}

int PbfTrack::get_next_track_id() {
    int ret_track_id = _s_track_idx;
    if (_s_track_idx == INT_MAX) {
        _s_track_idx = 0;
    } else {
        _s_track_idx++;
    }
    return ret_track_id;
}

bool PbfTrack::able_to_publish() {

    AINFO << _s_publish_if_has_lidar << " " << _invisible_in_lidar << " " << _lidar_objects.size();
    double invisible_period_threshold = 0.001;
    if (_invisible_period > invisible_period_threshold &&
        _invisible_in_lidar && _invisible_in_radar) {
        ADEBUG << "unable_to_publish: invisible " << _invisible_period;
        return false;
    }

    if (_s_publish_if_has_lidar && !_invisible_in_lidar && !_lidar_objects.empty()) {
        return true;
    }

    PbfSensorObjectPtr radar_object = get_latest_radar_object();
    if (_s_publish_if_has_radar && !_invisible_in_radar && radar_object != nullptr) {
        if (radar_object->sensor_type == RADAR) {
            if (radar_object->object->radar_supplement->range > _s_min_radar_confident_distance &&
                radar_object->object->radar_supplement->angle < _s_max_radar_confident_angle) {
                if (_fused_object->object->velocity.dot(_fused_object->object->direction) < 0.3) {
                    _fused_object->object->velocity.setZero();
                }
                return true;
            }
        }
    }
    return false;
}

void PbfTrack::update_measurements_life_with_measurement(
    std::map<std::string, PbfSensorObjectPtr>& objects,
    const std::string& sensor_id, double timestamp, double max_invisible_time) {
    for (auto it = objects.begin(); it != objects.end(); ) {
        if (sensor_id != it->first) {
            double period = timestamp - it->second->timestamp;
            if (period > max_invisible_time) {
                it->second = nullptr;
                it = objects.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }
}

void PbfTrack::update_measurements_life_without_measurement(
    std::map<std::string, PbfSensorObjectPtr>& objects,
    const std::string& sensor_id, double timestamp, double max_invisible_time,
    bool& invisible_state) {

    invisible_state = true;
    for (auto it = objects.begin(); it != objects.end(); ) {
        if (sensor_id == it->first) {
            it->second = nullptr; //TODO: consider in-view state
            it = objects.erase(it);
        } else {
            double period = timestamp - it->second->timestamp;
            if (period > max_invisible_time) {
                it->second = nullptr;
                it = objects.erase(it);
            } else {
                invisible_state = false;
                ++it;
            }
        }
    }
}

PbfSensorObjectPtr PbfTrack::get_latest_lidar_object() {
    PbfSensorObjectPtr lidar_object;
    for (auto it = _lidar_objects.begin(); it != _lidar_objects.end(); ++it) {
        if (lidar_object == nullptr) {
            lidar_object = it->second;
        } else if (lidar_object->timestamp < it->second->timestamp) {
            lidar_object = it->second;
        }
    }
    return lidar_object;
}

PbfSensorObjectPtr PbfTrack::get_latest_radar_object() {
    PbfSensorObjectPtr radar_object;
    for (auto it = _radar_objects.begin(); it != _radar_objects.end(); ++it) {
        if (radar_object == nullptr) {
            radar_object = it->second;
        } else if (radar_object->timestamp < it->second->timestamp) {
            radar_object = it->second;
        }
    }
    return radar_object;
}

PbfSensorObjectPtr PbfTrack::get_sensor_object(const SensorType& sensor_type,
    const std::string& sensor_id) {
    if (is_lidar(sensor_type)) {
        return get_lidar_object(sensor_id);
    } else if (is_radar(sensor_type)) {
        return get_radar_object(sensor_id);
    }
}

} // namespace perception
} // namespace apollo
