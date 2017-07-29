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

#include "modules/perception/obstacle/lidar/visualizer/pcl_obstacle_visualizer.h"

#include <gflags/gflags.h>

namespace apollo {
namespace perception {

DEFINE_int32(visual_spin_rate, 1, "visual_spin_rate");

int ObstacleTrackData::_s_max_queue_size = 300;

ObstacleTrackData::ObstacleTrackData() {

}

ObstacleTrackData::~ObstacleTrackData() {

}

void ObstacleTrackData::add_obstacle(const ObjectPtr& obj) {
    if (_history_data.size() >= (size_t)_s_max_queue_size) {
        _history_data.pop_front();
    }
    ObjectPtr new_obj(new Object());
    new_obj->clone(*obj);
    _history_data.push_back(new_obj);
    _consecutive_invisible_count = 0;
    _updated = true;
}

PCLObstacleVisualizer::PCLObstacleVisualizer():
        _name("ObsVisualizer"),
        _object_color_type(OBJECT_COLOR_TRACK),
        _capture_screen(false),
        _output_directory("") {
    initialize_pcl_visualizer();
}

PCLObstacleVisualizer::PCLObstacleVisualizer(
        const std::string& name,
        const OBJECT_COLOR_TYPE color_type,
        const bool capture_screen,
        const std::string& output_directory):
        _name(name),
        _object_color_type(color_type),
        _capture_screen(capture_screen),
        _output_directory(output_directory) {
    initialize_pcl_visualizer();
}

PCLObstacleVisualizer::~PCLObstacleVisualizer() {

}

//default pcl visualizer initialization
void PCLObstacleVisualizer::initialize_pcl_visualizer() {
    _pcl_vs = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
        new pcl::visualization::PCLVisualizer(_name));
    _view_port = 0;
    _pcl_vs->addCoordinateSystem(1.0, _view_port);
    set_size(2000, 1400);
    set_background_color(0.05, 0.05, 0.05, 0.0f);
    set_velodyne_height(2.5);
    set_car_points();
    set_camera_position();
}

void PCLObstacleVisualizer::set_size(int w, int h) {
    _width = w;
    _height = h;
    _pcl_vs->setSize(w, h);
}

void PCLObstacleVisualizer::set_background_color(float r, float g, float b, float a) {
    _pcl_vs->setBackgroundColor(r, g, b, a);
}

void PCLObstacleVisualizer::set_velodyne_height(float h) {
    _velodyne_height = h;
}

void PCLObstacleVisualizer::set_camera_position() {
    _up.x = 0;
    _up.y = 1;
    _up.z = 0;
    _view_point.x = 0;
    _view_point.y = 0;
    _view_point.z = 0;
    _camera_center.x = 0;
    _camera_center.y = 0;
    _camera_center.z = 150;
    // _camera_center = _main_car_points_local.at(7);
}

void PCLObstacleVisualizer::set_car_points() {
    _main_car_points_local.resize(9);
    _main_car_points_local.at(0).x = 0;
    _main_car_points_local.at(0).y = 0;
    _main_car_points_local.at(0).z = 0;

    _main_car_points_local.at(1).x = 0;
    _main_car_points_local.at(1).y = 0;
    _main_car_points_local.at(1).z = -_velodyne_height;
    _main_car_points_local.at(2).x = 3;
    _main_car_points_local.at(2).y = 0;
    _main_car_points_local.at(2).z = -_velodyne_height;

    _main_car_points_local.at(3).x = 2.5;
    _main_car_points_local.at(3).y = 1.0;
    _main_car_points_local.at(3).z = -_velodyne_height;
    _main_car_points_local.at(4).x = 2.5;
    _main_car_points_local.at(4).y = -1.0;
    _main_car_points_local.at(4).z = -_velodyne_height;
    _main_car_points_local.at(5).x = -2.5;
    _main_car_points_local.at(5).y = -1.0;
    _main_car_points_local.at(5).z = -_velodyne_height;
    _main_car_points_local.at(6).x = -2.5;
    _main_car_points_local.at(6).y = 1.0;
    _main_car_points_local.at(6).z = -_velodyne_height;

    _main_car_points_local.at(7).x = 0;
    _main_car_points_local.at(7).y = 0;
    _main_car_points_local.at(7).z = 160;
    _main_car_points_local.at(8).x = -40;
    _main_car_points_local.at(8).y = 0;
    _main_car_points_local.at(8).z = 50;
}

Eigen::Vector3f PCLObstacleVisualizer::get_cls_color(int cls) {
    Eigen::Vector3f rgb(255, 255, 255);
    switch (cls) {
        case 0:
            rgb << 128, 0, 255; //紫
            break;
        case 1:
            rgb << 0, 255, 255; //青
            break;
        case 2:
            rgb << 255, 255, 0; //黄
            break;
        case 3:
            rgb << 255, 128, 128; //赤
            break;
        case 4:
            rgb << 0, 0, 255; //蓝
            break;
        case 5:
            rgb << 0, 255, 0; //绿
            break;
        case 6:
            rgb << 255, 128, 0; //橙
            break;
        case 7:
            rgb << 255, 0, 0; //赤
            break;
    }
    return rgb;
}

Eigen::Vector3f PCLObstacleVisualizer::get_track_color(int track_id) {
    int cid = track_id % 124 + 1;
    int r = cid % 5;
    int g = ((cid - r) / 5) % 5;
    int b = (cid - r - 5 * g) / 25;
    float id2intesity[5] = {0, 128, 255, 64, 192};

    return Eigen::Vector3f(id2intesity[r], id2intesity[g], id2intesity[b]);
}

void PCLObstacleVisualizer::add_object(ObjectPtr& obj, std::string id) {
    Eigen::Vector3f coord_dir(1.0, 0.0, 0.0);
    Eigen::Vector3f size_dir = obj->direction.cast<float>();
    Eigen::Matrix3f rot = vector_rot_mat_2d_xy\
            <Eigen::Vector3f, Eigen::Matrix3f>(size_dir, coord_dir);

    Eigen::Vector3f cnt = obj->center.cast<float>();
    Eigen::Vector3f size = Eigen::Vector3f(obj->length, obj->width, obj->height);
    Eigen::Vector3f rgb = get_cls_color(obj->type) / 255;
    add_bbox_3d<pcl_util::Point>(cnt, size, rot, rgb, _pcl_vs.get(), _view_port, id);
}

void PCLObstacleVisualizer::visualize_detection(
        pcl_util::PointCloudPtr ground,
        std::vector<ObjectPtr>& objects,
        int frame_id) {

    add_pointcloud<pcl_util::Point>(_pcl_vs.get(), ground, 180, 180, 180, 1, "ground", _view_port);
    add_pointcloud<pcl_util::Point>(_pcl_vs.get(), \
            _main_car_points_local.makeShared(), 255, 255, 255, 5, "car_points", _view_port);

    for (size_t i = 0; i < objects.size(); ++i) {
        std::ostringstream oss;
        oss << "obj_" << i;
        Eigen::Vector3f rgb(1, 0, 0);
        add_pointcloud<pcl_util::Point>(_pcl_vs.get(), objects[i]->cloud, 255, 0,
                            0, 3, oss.str(), _view_port);
        add_object(objects[i], oss.str());
    }
    _pcl_vs.get()->setCameraPosition(_camera_center.x, _camera_center.y, _camera_center.z,
        _view_point.x, _view_point.y, _view_point.z, _up.x, _up.y, _up.z);
    _pcl_vs.get()->spinOnce();

    std::ostringstream oss_prefix;
    oss_prefix << std::setfill('0') << std::setw(6) << frame_id;
    std::string file_name = _output_directory + "/detection_" + oss_prefix.str() + ".png";
    std::cout << file_name << "\n";
    _pcl_vs.get()->saveScreenshot(file_name);
    _pcl_vs.get()->removeAllPointClouds();
    _pcl_vs.get()->removeAllShapes();
}

void PCLObstacleVisualizer::visualize_tracking(
        pcl_util::PointDCloudPtr cloud_world,
        std::vector<ObjectPtr>& objects_world,
        const Eigen::Matrix4d& pose_velo2w,
        int frame_id,
        const pcl_util::PointIndices& roi_indices) {
    Eigen::Matrix4d pose = pose_velo2w;
    if (_track_data.empty()) {
        _global_offset = Eigen::Vector3d(-pose_velo2w(0, 3),
            -pose_velo2w(1, 3), -pose_velo2w(2, 3));
    }
    pose(0, 3) += _global_offset[0];
    pose(1, 3) += _global_offset[1];
    pose(2, 3) += _global_offset[2];

    for (size_t i = 0; i < cloud_world->size(); i++) {
        cloud_world->points[i].x += _global_offset[0];
        cloud_world->points[i].y += _global_offset[1];
        cloud_world->points[i].z += _global_offset[2];
    }

    for (size_t i = 0; i <objects_world.size(); i++) {
        objects_world[i]->center += _global_offset;
        for (size_t j = 0; j < objects_world[i]->cloud->size(); j++) {
            objects_world[i]->cloud->points[j].x += _global_offset[0];
            objects_world[i]->cloud->points[j].y += _global_offset[1];
            objects_world[i]->cloud->points[j].z += _global_offset[2];
        }
        for (size_t j = 0; j < objects_world[i]->polygon.size(); j++) {
            objects_world[i]->polygon.points[j].x += _global_offset[0];
            objects_world[i]->polygon.points[j].y += _global_offset[1];
            objects_world[i]->polygon.points[j].z += _global_offset[2];
        }
    }

    /// transform camera center to world
    pcl_util::Point camera_center_w;
    transform_point_cloud<pcl_util::Point>(_camera_center, camera_center_w, pose);
    /// transform camera view point to world
    pcl_util::Point camera_vpoint_w;
    transform_point_cloud<pcl_util::Point>(_view_point, camera_vpoint_w, pose);
    /// transform host car points to world
    pcl_util::PointCloudPtr w_car_points(new pcl_util::PointCloud);
    transform_point_cloud<pcl_util::Point>(_main_car_points_local, *w_car_points, pose);
    /// show and save tracking result
    Eigen::Vector4d up_w(_up.x, _up.y, _up.z, 0);
    up_w = pose * up_w;

    std::ostringstream oss_prefix;
    oss_prefix << std::setfill('0') << std::setw(6) << frame_id;
    std::string file_name = _output_directory + "/tracking_" + oss_prefix.str() + ".png";

    pcl_util::PointDCloudPtr roi_cloud(new pcl_util::PointDCloud);
    pcl::copyPointCloud(*cloud_world, roi_indices, *roi_cloud);

    add_pointcloud<pcl_util::PointD>(_pcl_vs.get(), \
            cloud_world, 180, 180, 180, 1, "ground", _view_port);
    add_pointcloud<pcl_util::Point>(_pcl_vs.get(), \
            w_car_points, 255, 255, 255, 5, "car_points", _view_port);
    add_pointcloud<pcl_util::PointD>(_pcl_vs.get(), \
            roi_cloud, 0, 0, 205, 1, "roi", _view_port); // MediumBlue

    for (std::map<int, ObstacleTrackData>::iterator it = _track_data.begin();
        it != _track_data.end(); ++it) {
        it->second._updated = false;
    }

    for (size_t i = 0; i < objects_world.size(); i++) {
        int track_id = objects_world[i]->track_id;
        std::map<int, ObstacleTrackData>::iterator it = _track_data.find(track_id);
        if (it == _track_data.end()) {
            ObstacleTrackData& data = _track_data[track_id];
            data.add_obstacle(objects_world[i]);
        } else {
            it->second.add_obstacle(objects_world[i]);
        }
    }

    for (std::map<int, ObstacleTrackData>::iterator it = _track_data.begin();
        it != _track_data.end();) {
        if (it->second._updated == false) {
            it->second._consecutive_invisible_count++;
            if (it->second._consecutive_invisible_count > 1) {
                it = _track_data.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    for (std::map<int, ObstacleTrackData>::iterator it = _track_data.begin();
        it != _track_data.end(); ++it) {
        if (it->second._consecutive_invisible_count > 0) {
            continue;
        }

        std::ostringstream oss;
        oss << "track_" << it->first;
        std::string id = oss.str() + "_cloud";
        ObjectPtr recent_obj = it->second._history_data.back();
        add_pointcloud<pcl_util::Point>(_pcl_vs.get(),
            recent_obj->cloud, 255, 0, 3, 1, id, _view_port);

        add_object(recent_obj, oss.str());

        pcl_util::Point src;
        pcl_util::Point dst;
        src.x = recent_obj->center[0];
        src.y = recent_obj->center[1];
        src.z = recent_obj->center[2];

        dst.x = src.x + recent_obj->velocity[0];
        dst.y = src.y + recent_obj->velocity[1];
        dst.z = src.z + recent_obj->velocity[2];

        id = oss.str() + "_velocity";
        _pcl_vs.get()->addLine(src, dst, 1, 1, 1, id);

        std::vector<Eigen::Vector3d> history_centers;
        std::deque<ObjectPtr>::iterator dq_it = it->second._history_data.begin();
        for (; dq_it != it->second._history_data.end(); dq_it++) {
            history_centers.push_back((*dq_it)->center);
        }

        Eigen::Vector3f rgb(1, 1, 0);
        int history_length = history_centers.size();
        for (int j = 0; j < history_length - 1; j++) {
            std::ostringstream id_oss;
            id_oss << "_edge_" << j;
            std::string tag = oss.str() + id_oss.str();
            src.x = history_centers[j][0];
            src.y = history_centers[j][1];
            src.z = history_centers[j][2];

            dst.x = history_centers[j + 1][0];
            dst.y = history_centers[j + 1][1];
            dst.z = history_centers[j + 1][2];
            _pcl_vs.get()->addLine(src, dst, rgb[0], rgb[1], rgb[2], tag);
        }
        pcl_util::Point loc;
        loc.x = recent_obj->center[0];
        loc.y = recent_obj->center[1];
        loc.z = recent_obj->center[2];
        std::string text_id = oss.str();
        std::ostringstream text_oss;
        text_oss << recent_obj->track_id;
        std::string disp_text = text_oss.str();
        _pcl_vs.get()->addText3D(disp_text, loc, 0.5, 1, 1, 1, text_id + "_text", _view_port);
    }

    _pcl_vs.get()->setCameraPosition(camera_center_w.x, camera_center_w.y, camera_center_w.z,
            camera_vpoint_w.x, camera_vpoint_w.y, camera_vpoint_w.z, up_w[0], up_w[1], up_w[2]);

    _pcl_vs.get()->spinOnce(FLAGS_visual_spin_rate);
    _pcl_vs.get()->saveScreenshot(file_name);
    _pcl_vs.get()->removeAllPointClouds();
    _pcl_vs.get()->removeAllShapes();
}

}  // namespace perception
}  // namespace apollo
