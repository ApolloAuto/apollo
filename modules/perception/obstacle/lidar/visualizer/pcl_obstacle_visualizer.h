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

#ifndef MODULES_PERCEPTION_VISUALIZATION_PCL_OBSTACLE_VISUALIZER_H
#define MODULES_PERCEPTION_VISUALIZATION_PCL_OBSTACLE_VISUALIZER_H

#include <deque>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/visualizer/pcl_vis_util.h"

namespace apollo {
namespace perception {

enum OBJECT_COLOR_TYPE{
    OBJECT_COLOR_CLASSIFICATION = 0,
    OBJECT_COLOR_TRACK
};

class ObstacleTrackData{
public:
    ObstacleTrackData();
    ~ObstacleTrackData();

    void add_obstacle(const ObjectPtr& obj);

public:
    std::deque<ObjectPtr>       _history_data;
    int                         _consecutive_invisible_count;
    bool                        _updated;
private:
    static int                  _s_max_queue_size;
};

class PCLObstacleVisualizer{
public:
    PCLObstacleVisualizer();
    explicit PCLObstacleVisualizer(
        const std::string& name,
        const OBJECT_COLOR_TYPE color_type,
        const bool capture_screen,
        const std::string& output_directory);
    ~PCLObstacleVisualizer();

    void set_size(int w, int h);
    void set_background_color(float r, float g, float b, float a = 1.0f);
    void set_velodyne_height(float height);
    void set_car_points();
    void set_camera_position();
    void set_output_dir(const std::string& outptut_dir) {
        _output_directory = outptut_dir;
    }

    void visualize_detection(pcl_util::PointCloudPtr ground,
        std::vector<ObjectPtr>& objects, int frame_id = -1);

    void visualize_tracking(
            pcl_util::PointDCloudPtr cloud_world,
            std::vector<ObjectPtr>& objects_world,
        const Eigen::Matrix4d& pose_velo2w,
        int frame_id,
        const pcl_util::PointIndices& roi_indices);

protected:
    void initialize_pcl_visualizer();
    Eigen::Vector3f get_cls_color(int cls);
    Eigen::Vector3f get_track_color(int track_id);

    void add_object(ObjectPtr& obj, std::string id);

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _pcl_vs;

    std::map<int, ObstacleTrackData> _track_data;

    std::string _name;
    int _view_port;

    pcl_util::Point _camera_center;
    pcl_util::Point _view_point;
    pcl_util::Point _up;

    float _velodyne_height;
    pcl_util::PointCloud _main_car_points_local;

    int _width;
    int _height;

    OBJECT_COLOR_TYPE _object_color_type;
    bool _capture_screen;
    std::string _output_directory;

    Eigen::Vector3d _global_offset;
};

}  // namespace perception
}  // namespace apollo
#endif  // MODULES_PERCEPTION_VISUALIZATION_PCL_OBSTACLE_VISUALIZER_H
