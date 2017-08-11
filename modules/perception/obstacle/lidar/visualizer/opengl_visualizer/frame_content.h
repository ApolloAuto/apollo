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

#ifndef ADU_PERCEPTION_OBSTACLE_FRAME_CONTENT_H
#define ADU_PERCEPTION_OBSTACLE_FRAME_CONTENT_H
#include <deque>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <thread>

#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {
 
class FrameContent {
public:
    FrameContent();
    ~FrameContent();
    
    void set_lidar_pose(const Eigen::Matrix4d& pose);
    Eigen::Matrix4d get_pose_v2w();

    void set_lidar_cloud(pcl_util::PointCloudPtr cloud);
    void set_lidar_roi_cloud(pcl_util::PointCloudPtr cloud);
    pcl_util::PointCloudPtr get_cloud();
    pcl_util::PointCloudPtr get_roi_cloud();

    bool has_cloud();

    void set_tracked_objects(const std::vector<ObjectPtr>& objects);                                  

    std::vector<ObjectPtr> get_tracked_objects();                         

protected:
    //coordinate transform utilities
    void offset_pointcloud(pcl_util::PointCloud& cloud, const Eigen::Vector3d& offset);
    void offset_pointcloud(pcl_util::PointDCloud& cloud, const Eigen::Vector3d& offset);
    void offset_object(ObjectPtr object, const Eigen::Vector3d& offset);
private:
    //input
    //1.lidar
    Eigen::Matrix4d _pose_v2w;
    pcl_util::PointCloudPtr _cloud;
    pcl_util::PointCloudPtr _roi_cloud;

    Eigen::Vector3d _global_offset;  
    bool _global_offset_initialized;
    std::vector<ObjectPtr> _tracked_objects_lidar;    //after tracking
};

} // namespace perception
} // namespace apollo

#endif //  ADU_PERCEPTION_OBSTACLE_FRAME_CONTENT_H
