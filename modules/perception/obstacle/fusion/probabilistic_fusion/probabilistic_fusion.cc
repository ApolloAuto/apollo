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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.h"

#include <iomanip>
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/common/macro.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {

ProbabilisticFusion::ProbabilisticFusion() : _publish_sensor_id("velodyne_64"),
    _started(false), _matcher(nullptr), _sensor_manager(nullptr), _track_manager(nullptr),
    _use_radar(true), _use_lidar(true) {
}

ProbabilisticFusion::~ProbabilisticFusion() {
    if (_matcher) {
        delete _matcher;
        _matcher = nullptr;
    }
}

bool ProbabilisticFusion::init() {
    using apollo::perception::ConfigManager;
    using apollo::perception::ModelConfig;
    _sensor_manager = PbfSensorManager::instance();
    ACHECK(_sensor_manager != nullptr) << "Failed to get PbfSensorManager instance";
    _track_manager = PbfTrackManager::instance();
    ACHECK(_track_manager != nullptr) << "Failed to get PbfTrackManager instance";
    const ModelConfig* model_config = NULL;
    if (!ConfigManager::instance()->GetModelConfig(name(), &model_config)) {
        AERROR << "not found model config: " << name();
        return false;
    }
    /**matching parameters*/
    std::string match_method = "hm_matcher";
    if (!model_config->GetValue("match_method", &match_method)) {
        AERROR << "match_method not found";
    }
    if (match_method == "hm_matcher") {
        _matcher = new PbfHmTrackObjectMatcher();
        if (_matcher->init()) {
            AINFO << "Initialize " << _matcher->name() << " successfully!";
        } else {
            AERROR << "Failed to initialize " << _matcher->name();
            return false;
        }
    } else {
        AERROR << "undefined match_method " << match_method << " and use default hm_matcher";
        _matcher = new PbfHmTrackObjectMatcher();
        if (_matcher->init()) {
            AINFO << "Initialize " << _matcher->name() << " successfully!";
        } else {
            AERROR << "Failed to initialize " << _matcher->name();
            return false;
        }
    }

    float max_match_distance = 4.0;
    if (!model_config->GetValue("max_match_distance", &max_match_distance)) {
        AERROR << "max_match_distance not found";
    }
    AINFO << "Probabilistic_fusion max_match_distance: " << max_match_distance;
    PbfBaseTrackObjectMatcher::set_max_match_distance(max_match_distance);

    /**track related parameters*/
    float max_lidar_invisible_period = 0.25;
    float max_radar_invisible_period = 0.25;
    if (!model_config->GetValue("max_lidar_invisible_period", &max_lidar_invisible_period)) {
        AERROR << "max_lidar_invisible_period not found";
    }
    PbfTrack::set_max_lidar_invisible_period(max_lidar_invisible_period);
    AINFO << "max_lidar_invisible_period: " << max_lidar_invisible_period;

    if (!model_config->GetValue("max_radar_invisible_period", &max_radar_invisible_period)) {
        AERROR << "max_radar_invisible_period not found";
    }
    PbfTrack::set_max_radar_invisible_period(max_radar_invisible_period);
    AINFO << "max_radar_invisible_period: " << max_radar_invisible_period;

    float max_radar_confident_angle = 30;
    float min_radar_confident_distance = 40;
    if (!model_config->GetValue("max_radar_confident_angle", &max_radar_confident_angle)) {
        AERROR << "max_radar_confident_angle not found";
    }
    PbfTrack::set_max_radar_confident_angle(max_radar_confident_angle);
    AINFO << "max_radar_confident_angle: " << max_radar_confident_angle;

    if (!model_config->GetValue("min_radar_confident_distance", &min_radar_confident_distance)) {
        AERROR << "min_radar_confident_distance not found";
    }
    PbfTrack::set_min_radar_confident_distance(min_radar_confident_distance);
    AINFO << "min_radar_confident_distance: " << min_radar_confident_distance;

    bool publish_if_has_lidar = true;
    if (!model_config->GetValue("publish_if_has_lidar", &publish_if_has_lidar)) {
        AERROR << "publish_if_has_lidar not found";
    }
    PbfTrack::set_publish_if_has_lidar(publish_if_has_lidar);
    AINFO << "publish_if_has_lidar: " << (publish_if_has_lidar ? "true" : "false");

    bool publish_if_has_radar = true;
    if (!model_config->GetValue("publish_if_has_radar", &publish_if_has_radar)) {
        AERROR << "publish_if_has_radar not found";
    }
    PbfTrack::set_publish_if_has_radar(publish_if_has_radar);
    AINFO << "publish_if_has_radar: " << (publish_if_has_radar ? "true" : "false");

    /**publish driven*/
    if (!model_config->GetValue("publish_sensor", &_publish_sensor_id)) {
        AERROR << "publish_sensor not found";
    }
    if (_publish_sensor_id != "velodyne_64" && _publish_sensor_id != "radar") {
        AERROR << "Invalid publish_sensor value: " << _publish_sensor_id;
    }
    AINFO << "publish_sensor: " << _publish_sensor_id;

    if (!model_config->GetValue("use_radar", &_use_radar)) {
        AERROR << "use_radar not found";
    }
    AINFO << "use_radar :" << _use_radar;
    if (!model_config->GetValue("use_lidar", &_use_lidar)) {
        AERROR << "use_lidar not found";
    }
    AINFO << "use_lidar:" << _use_lidar;
    AINFO << "ProbabilisticFusion initialize successfully";
    return true;
}

bool ProbabilisticFusion::fuse(const std::vector<SensorObjects>& multi_sensor_objects,
    std::vector<ObjectPtr>* fused_objects) {
    ACHECK(fused_objects != nullptr) << "parameter fused_objects is nullptr";

    std::vector<PbfSensorFramePtr> frames;
    double fusion_time = 0;
    {
        _sensor_data_rw_mutex.lock();
        bool need_to_fusion = false;
        //1. collect sensor objects data
        for (size_t i = 0; i < multi_sensor_objects.size(); ++i) {
            auto sensor_type = multi_sensor_objects[i].sensor_type;
            AINFO << "add sensor measurement: " << GetSensorType(sensor_type)
                << ", obj_cnt : " << multi_sensor_objects[i].objects.size()
                << ", " << std::fixed << std::setprecision(12)
                << multi_sensor_objects[i].timestamp;
            if (is_lidar(sensor_type) && !_use_lidar) {
                continue;
            }
            if (is_radar(sensor_type) && !_use_radar) {
                continue;
            }

            if (GetSensorType(multi_sensor_objects[i].sensor_type) == _publish_sensor_id) {
                need_to_fusion = true;
                fusion_time = multi_sensor_objects[i].timestamp;
                _started = true;
                _sensor_manager->add_sensor_measurements(multi_sensor_objects[i]);
            } else if (_started) {
                _sensor_manager->add_sensor_measurements(multi_sensor_objects[i]);
            }
        }

        if (!need_to_fusion) {
            _sensor_data_rw_mutex.unlock();
            return true;
        }

        //2.query related sensor frames for fusion
        _sensor_manager->get_latest_frames(fusion_time, &frames);
        _sensor_data_rw_mutex.unlock();
        AINFO << "Get " << frames.size() << " related frames for fusion";
    }

    {
        _fusion_mutex.lock();
        //3.peform fusion on related frames
        for (size_t i = 0; i < frames.size(); ++i) {
            fuse_frame(frames[i]);
        }

        //4.collect results
        collect_fused_objects(fusion_time, fused_objects);
        _fusion_mutex.unlock();
    }

    return true;
}

std::string ProbabilisticFusion::name() const {
    return "ProbabilisticFusion";
}

void ProbabilisticFusion::fuse_frame(const PbfSensorFramePtr& frame) {

    AINFO << "Fusing frame: " << frame->sensor_id << ","
        << "object_number: " << frame->objects.size() << ","
        << "timestamp: " << std::fixed << std::setprecision(12) << frame->timestamp;
    std::vector<PbfSensorObjectPtr>& objects = frame->objects;
    std::vector<PbfSensorObjectPtr> background_objects;
    std::vector<PbfSensorObjectPtr> foreground_objects;
    decompose_frame_objects(objects, foreground_objects, background_objects);

    Eigen::Vector3d ref_point = frame->sensor2world_pose.topRightCorner(3, 1);
    fuse_foreground_objects(foreground_objects,
        ref_point, frame->sensor_type, frame->sensor_id, frame->timestamp);
    fuse_background_objects(background_objects,
        frame->sensor_type, frame->sensor_id, frame->timestamp);

    _track_manager->remove_lost_tracks();

}

void ProbabilisticFusion::create_new_tracks(const std::vector<PbfSensorObjectPtr>& sensor_objects,
    const std::vector<int>& unassigned_ids) {
    for (int i = 0; i < (int)unassigned_ids.size(); i++) {
        int id = unassigned_ids[i];
        PbfTrackPtr track(new PbfTrack(sensor_objects[id]));
        _track_manager->add_track(track);
    }
}

void ProbabilisticFusion::update_assigned_tracks(std::vector<PbfTrackPtr>& tracks,
    std::vector<PbfSensorObjectPtr>& sensor_objects,
    std::vector<TrackObjectPair>& assignments,
    const std::vector<double>& track_object_dist) {

    for (int i = 0; i < (int)assignments.size(); i++) {
        int local_track_index = assignments[i].first;
        int local_obj_index = assignments[i].second;
        tracks[local_track_index]->update_with_sensor_object(sensor_objects[local_obj_index],
            track_object_dist[local_track_index]);
    }
}

void ProbabilisticFusion::update_unassigned_tracks(std::vector<PbfTrackPtr>& tracks,
    const std::vector<int>& unassigned_tracks,
    const std::vector<double>& track_object_dist,
    const SensorType& sensor_type,
    const std::string& sensor_id, double timestamp) {

    for (int i = 0; i < (int)unassigned_tracks.size(); i++) {
        int local_track_index = unassigned_tracks[i];
        tracks[local_track_index]->update_without_sensor_object(
                sensor_type, sensor_id, track_object_dist[local_track_index], timestamp);
    }
}

void ProbabilisticFusion::collect_fused_objects(double timestamp,
    std::vector<ObjectPtr>* fused_objects) {
    if (fused_objects == nullptr) {
        return;
    }
    fused_objects->clear();

    int fg_obj_num = 0;
    std::vector<PbfTrackPtr>& tracks = _track_manager->get_tracks();
    for (size_t i = 0; i < tracks.size(); i++) {
        if (tracks[i]->able_to_publish()) {
            PbfSensorObjectPtr fused_object = tracks[i]->get_fused_object();
            ObjectPtr obj(new Object());
            obj->clone(*(fused_object->object));
            obj->track_id = tracks[i]->get_track_id();
            obj->latest_tracked_time = timestamp;
            obj->tracking_time = tracks[i]->get_tracking_period();
            // obj->confidence = tracks[i]->get_toic_probability();
            fused_objects->push_back(obj);
            fg_obj_num++;
        }
    }

    int bg_obj_num = 0;
    std::vector<PbfBackgroundTrackPtr>& background_tracks =
        _track_manager->get_background_tracks();
    for (size_t i = 0; i < background_tracks.size(); i++) {
        if (background_tracks[i]->able_to_publish()) {
            PbfSensorObjectPtr fused_object = background_tracks[i]->get_fused_object();
            ObjectPtr obj(new Object());
            obj->clone(*(fused_object->object));
            obj->track_id = background_tracks[i]->get_track_id();
            obj->latest_tracked_time = timestamp;
            obj->tracking_time = background_tracks[i]->get_tracking_period();
            fused_objects->push_back(obj);
            bg_obj_num++;
        }
    }

    AINFO << "fg_track_cnt = " << tracks.size()
        << ", bg_track_cnt = " << background_tracks.size();
    AINFO << "collect objects : fg_obj_cnt = " << fg_obj_num
        << ", bg_obj_cnt = " << bg_obj_num << ", timestamp = " << GLOG_TIMESTAMP(timestamp);
}

void ProbabilisticFusion::decompose_frame_objects(
    const std::vector<PbfSensorObjectPtr>& frame_objects,
    std::vector<PbfSensorObjectPtr>& foreground_objects,
    std::vector<PbfSensorObjectPtr>& background_objects) {

    foreground_objects.clear();
    background_objects.clear();
    for (size_t i = 0; i < frame_objects.size(); i++) {
        if (frame_objects[i]->object->is_background) {
            background_objects.push_back(frame_objects[i]);
        } else {
            foreground_objects.push_back(frame_objects[i]);
        }
    }
}

void ProbabilisticFusion::fuse_background_objects(
    std::vector<PbfSensorObjectPtr>& background_objects,
    const SensorType& sensor_type,
    const std::string& sensor_id,
    double timestamp) {

    std::vector<PbfBackgroundTrackPtr> bg_tracks = _track_manager->get_background_tracks();
    std::map<int, int> local_id_2_track_ind_map;
    std::vector<int> track_tag(bg_tracks.size(), 0);
    std::vector<int> object_tag(background_objects.size(), 0);
    std::vector<TrackObjectPair> assignments;

    for (size_t i = 0; i < bg_tracks.size(); i++) {
        const PbfSensorObjectPtr& obj = bg_tracks[i]->get_fused_object();
        int l_id = obj->object->track_id;
        local_id_2_track_ind_map[l_id] = i;
    }
    for (size_t i = 0; i < background_objects.size(); i++) {
        int l_id = background_objects[i]->object->track_id;
        auto it = local_id_2_track_ind_map.find(l_id);
        if (it != local_id_2_track_ind_map.end()) {
            size_t track_ind = it->second;
            assignments.push_back(std::make_pair<int, int>(track_ind, i));
            track_tag[track_ind] = 1;
            object_tag[i] = 1;
            continue;
        }
    }

    //update assigned
    for (size_t i = 0; i < assignments.size(); i++) {
        int track_ind = assignments[i].first;
        int obj_ind = assignments[i].second;
        bg_tracks[track_ind]->update_with_sensor_object(background_objects[obj_ind]);
    }

    //update unassigned
    for (size_t i = 0; i < track_tag.size(); i++) {
        if (track_tag[i] == 0) {
            bg_tracks[i]->update_without_sensor_object(sensor_type, sensor_id, timestamp);
        }
    }

    //crete new tracks
    for (size_t i = 0; i < object_tag.size(); i++) {
        if (object_tag[i] == 0) {
            PbfBackgroundTrackPtr bg_track(new PbfBackgroundTrack(background_objects[i]));
            _track_manager->add_background_track(bg_track);
        }
    }
}

void ProbabilisticFusion::fuse_foreground_objects(
    std::vector<PbfSensorObjectPtr>& foreground_objects,
    Eigen::Vector3d ref_point,
    const SensorType& sensor_type,
    const std::string& sensor_id,
    double timestamp) {

    std::vector<int> unassigned_tracks;
    std::vector<int> unassigned_objects;
    std::vector<TrackObjectPair> assignments;

    std::vector<PbfTrackPtr>& tracks = _track_manager->get_tracks();

    TrackObjectMatcherOptions options;
    options.ref_point = &ref_point;

    std::vector<double> track2measurements_dist;
    std::vector<double> measurement2tracks_dist;
    _matcher->match(tracks, foreground_objects, options, assignments,
            unassigned_tracks, unassigned_objects,
            track2measurements_dist, measurement2tracks_dist);

    AINFO << "fg_track_cnt = " << tracks.size() << ", fg_obj_cnt = " << foreground_objects.size()
        << ", assignement = " << assignments.size() << ", unassigned_track_cnt = "
        << unassigned_tracks.size() << ", unassigned_obj_cnt = " << unassigned_objects.size();

    update_assigned_tracks(tracks, foreground_objects, assignments, track2measurements_dist);

    update_unassigned_tracks(tracks, unassigned_tracks, track2measurements_dist,
        sensor_type, sensor_id, timestamp);

    create_new_tracks(foreground_objects, unassigned_objects);
}

// Register plugin.
REGISTER_FUSION(ProbabilisticFusion);

} // namespace perception
} // namespace apollo
