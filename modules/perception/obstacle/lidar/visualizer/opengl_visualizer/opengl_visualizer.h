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

#ifndef ADU_PERCEPTION_OBSTACLE_VISUALIZER_OPENGL_VISUALIZER_H
#define ADU_PERCEPTION_OBSTACLE_VISUALIZER_OPENGL_VISUALIZER_H
#include <string>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "modules/perception/lib/pcl_util/pcl_types.h"
/*#include "obstacle/lidar/visualizer/pcl_vis_util.h"
#include "obstacle/visualizer/base_visualizer.h"
*/
#include "frame_content.h"

#include "glfw_viewer.h"

namespace apollo {
namespace perception { 

class OpenglVisualizer {
public:
    OpenglVisualizer();
    virtual ~OpenglVisualizer() = default;

    virtual bool init();

    virtual std::string name() const {
        return _name;
}
    
    void update_camera_system(FrameContent* content);

    virtual void render(FrameContent& content);    

private:
    void set_size(int w, int h);
    void set_background_color(float r, float g, float b, float a);
    void set_velodyne_height(float h);
    void set_main_car_points();
    void set_camera_position();

    float _velodyne_height;
    pcl_util::Point _camera_center_velodyne;
    pcl_util::Point _view_point_velodyne;
    pcl_util::Point _up_velodyne;
    pcl_util::Point _forward_velodyne;
    pcl_util::PointCloud _main_car_points_velodyne;
    
    pcl_util::Point _camera_center_world;
    pcl_util::Point _view_point_world;
    pcl_util::Point _up_world;
    pcl_util::Point _forward_world;
    pcl_util::PointCloud _main_car_points_world;

    boost::shared_ptr<GLFWViewer> _opengl_vs;
    std::string _name;
    bool _init = false;

};


} // namespace perception
} // namespace adu

#endif //  ADU_PERCEPTION_OBSTACLE_VISUALIZER_PCL_VISUALIZER_H
