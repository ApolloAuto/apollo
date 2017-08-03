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

#include "opengl_visualizer.h"
#include "modules/common/log.h"
#include "modules/perception/obstacle/common/geometry_util.h"

namespace apollo {
namespace perception {

OpenglVisualizer::OpenglVisualizer() : _name("OpenglVisualizer") {
}

bool OpenglVisualizer::init() {
    _opengl_vs = boost::shared_ptr<GLFWViewer>(new GLFWViewer());
 
    if (_opengl_vs == nullptr) {
        AINFO << "Failed to create opengl viewer";
        return false;
    }

    if (_opengl_vs->initialize() == false) {
        AINFO << "Failed to initialize opengl viewer";
        return false;
    }

    set_size(2000, 1400);
    set_background_color(0.05, 0.05, 0.05, 0.0f);
    set_velodyne_height(2.5);
    set_main_car_points();
    set_camera_position();
    AINFO << "Initialize OpenglVisualizer successfully";
    return true;
}

void OpenglVisualizer::render(FrameContent& content) {  
    Eigen::Vector3d camera_world_pos(_camera_center_world.x, _camera_center_world.y, _camera_center_world.z);
    Eigen::Vector3d camera_scene_center(_view_point_world.x,    _view_point_world.y,    _view_point_world.z);
    Eigen::Vector3d camera_up_vector(_up_world.x, _up_world.y, _up_world.z);
    _opengl_vs->set_camera_para(Eigen::Vector3d(_camera_center_world.x, _camera_center_world.y, _camera_center_world.z), 
                                Eigen::Vector3d(_view_point_world.x, _view_point_world.y, _view_point_world.z), 
                                Eigen::Vector3d(_up_world.x, _up_world.y, _up_world.z)
                               );
    _opengl_vs->set_forward_dir(Eigen::Vector3d(_forward_world.x, _forward_world.y, _forward_world.z));
    _opengl_vs->set_frame_content(&content);
    _opengl_vs->spin_once();

   // AINFO << "OpenglVisualizer spin_once";
}

void OpenglVisualizer::set_size(int w, int h) { 
    _opengl_vs->set_size(w, h); 
}

void OpenglVisualizer::set_background_color(float r, float g, float b, float a) { 
    _opengl_vs->set_background_color(Eigen::Vector3d(r, g, b)); 
}

void OpenglVisualizer::set_velodyne_height(float h) { 
    _velodyne_height = h; 
}

void OpenglVisualizer::set_camera_position() { 
    _up_velodyne.x = 0;
    _up_velodyne.y = 1;
    _up_velodyne.z = 0;
    _forward_velodyne.x = 1;
    _forward_velodyne.y = 0;
    _forward_velodyne.z = 0;
    _view_point_velodyne.x = 0;
    _view_point_velodyne.y = 0;
    _view_point_velodyne.z = 0;
    _camera_center_velodyne.x = 0;
    _camera_center_velodyne.y = 0;
    _camera_center_velodyne.z = 100; 
}

void OpenglVisualizer::set_main_car_points() { 
    _main_car_points_velodyne.resize(9);
    _main_car_points_velodyne.at(0).x = 0;
    _main_car_points_velodyne.at(0).y = 0;
    _main_car_points_velodyne.at(0).z = 0;

    _main_car_points_velodyne.at(1).x = 0;
    _main_car_points_velodyne.at(1).y = 0;
    _main_car_points_velodyne.at(1).z = -_velodyne_height;
    _main_car_points_velodyne.at(2).x = 3;
    _main_car_points_velodyne.at(2).y = 0;
    _main_car_points_velodyne.at(2).z = -_velodyne_height;

    _main_car_points_velodyne.at(3).x = 2.5;
    _main_car_points_velodyne.at(3).y = 1.0;
    _main_car_points_velodyne.at(3).z = -_velodyne_height;
    _main_car_points_velodyne.at(4).x = 2.5;
    _main_car_points_velodyne.at(4).y = -1.0;
    _main_car_points_velodyne.at(4).z = -_velodyne_height;
    _main_car_points_velodyne.at(5).x = -2.5;
    _main_car_points_velodyne.at(5).y = -1.0;
    _main_car_points_velodyne.at(5).z = -_velodyne_height;
    _main_car_points_velodyne.at(6).x = -2.5;
    _main_car_points_velodyne.at(6).y = 1.0;
    _main_car_points_velodyne.at(6).z = -_velodyne_height;

    _main_car_points_velodyne.at(7).x = 0;
    _main_car_points_velodyne.at(7).y = 0;
    _main_car_points_velodyne.at(7).z = 160;
    _main_car_points_velodyne.at(8).x = -40;
    _main_car_points_velodyne.at(8).y = 0;
    _main_car_points_velodyne.at(8).z = 50; 
}


void OpenglVisualizer::update_camera_system(FrameContent* content) { 
    Eigen::Matrix4d pose_v2w = content->get_pose_v2w(); 

    transform_point_cloud<pcl_util::Point>(_camera_center_velodyne, 
        _camera_center_world, pose_v2w); 

    transform_point_cloud<pcl_util::Point>(_view_point_velodyne, 
        _view_point_world, pose_v2w); 

    transform_point_cloud<pcl_util::Point>(_main_car_points_velodyne, 
        _main_car_points_world, pose_v2w); 

    Eigen::Vector4d up_w(_up_velodyne.x, _up_velodyne.y, _up_velodyne.z, 0); 

    up_w = pose_v2w * up_w;
    _up_world.x = up_w[0];
    _up_world.y = up_w[1];
    _up_world.z = up_w[2]; 

    Eigen::Vector4d fd_w(_forward_velodyne.x, _forward_velodyne.y, _forward_velodyne.z, 0);
    fd_w = pose_v2w * fd_w;
    _forward_world.x = fd_w[0];
    _forward_world.y = fd_w[1];
    _forward_world.z = fd_w[2];
}


} // namespace perception
} // namespace adu
