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

#ifndef  MODULES_PERCEPTION_OBSTACLE_VISUALIZATION_PCL_VIS_UTIL_H
#define  MODULES_PERCEPTION_OBSTACLE_VISUALIZATION_PCL_VIS_UTIL_H
#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/common/geometry_util.h"

namespace apollo {
namespace perception {
/*get color based on object class id*/
Eigen::Vector3f get_cls_color(int cls);

/*get color for track id*/
Eigen::Vector3f get_track_color(int track_id);

void draw_boundingbox_3d(pcl::visualization::PCLVisualizer* p, int r, int g, int b, 
        int line_size, std::string id, int viewport, 
        const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt);

void draw_boundingbox_3d(pcl::visualization::PCLVisualizer* p, int r, int g, int b, 
        int line_size, std::string id, int viewport, const Eigen::Vector3f &min_pt, 
        const Eigen::Vector3f& max_pt, Eigen::Quaterniond& quat);

template <typename PointT>
void get_bbox_polygon(typename pcl::PointCloud<PointT>::Ptr bbox,
        Eigen::Matrix3f& rot_mat, Eigen::Vector3f& center, float half_wid, float half_len) {
    bbox->points.resize(4);

    Eigen::Vector3f local_pt;
    local_pt = Eigen::Vector3f(-half_wid, -half_len, 0);
    local_pt = rot_mat * local_pt;
    local_pt += center;
    bbox->points[0].x = local_pt[0];
    bbox->points[0].y = local_pt[1];
    bbox->points[0].z = local_pt[2];

    local_pt = Eigen::Vector3f(-half_wid, half_len, 0);
    local_pt = rot_mat * local_pt;
    local_pt += center;
    bbox->points[1].x = local_pt[0];
    bbox->points[1].y = local_pt[1];
    bbox->points[1].z = local_pt[2];

    local_pt = Eigen::Vector3f(half_wid, half_len, 0);
    local_pt = rot_mat * local_pt;
    local_pt += center;
    bbox->points[2].x = local_pt[0];
    bbox->points[2].y = local_pt[1];
    bbox->points[2].z = local_pt[2];

    local_pt = Eigen::Vector3f(half_wid, -half_len, 0);
    local_pt = rot_mat * local_pt;
    local_pt += center;
    bbox->points[3].x = local_pt[0];
    bbox->points[3].y = local_pt[1];
    bbox->points[3].z = local_pt[2];
}

template <typename PointT>
void add_pointcloud(pcl::visualization::PCLVisualizer *p, 
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int r, int g, int b, 
    int point_size, std::string id, int viewport) {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> handle(cloud, r, g, b);
    p->addPointCloud(cloud, handle, id, viewport);
    p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
            point_size, id, viewport);
}

template <typename PointT>
void add_pointcloud(pcl::visualization::PCLVisualizer *p, 
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        const std::vector<int> &indices, 
        int r, int g, int b, 
        int point_size, std::string id, int viewport) {
    typename pcl::PointCloud<PointT>::Ptr subcloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud, indices, *subcloud);
    add_pointcloud<PointT>(p, subcloud, r, g, b, point_size, id, viewport);
}
    
template <typename PointT>
void display_map_polygon(typename pcl::PointCloud<PointT>::Ptr map_polygon,
    pcl::visualization::PCLVisualizer* pcl_vs,
    int pcl_vp) {
    for (int i = 0; i < map_polygon->points.size(); ++i) {
        std::ostringstream oss_vet;
        oss_vet << "_vet_" << i;
        if (i == map_polygon->points.size() - 1) {
            pcl_vs->addLine<PointT, PointT>(map_polygon->points[i], map_polygon->points[0],
                    255, 255, 255, oss_vet.str(), pcl_vp);
            continue;
        }
        pcl_vs->addLine<PointT, PointT>(map_polygon->points[i], map_polygon->points[i + 1],
                    255, 255, 255, oss_vet.str(), pcl_vp);
    }
}

template <typename PointT>
void display_had_map(std::vector<typename pcl::PointCloud<PointT> > &map_polygons,
    pcl::visualization::PCLVisualizer* pcl_vs, 
    int pcl_vp) {
    typename pcl::PointCloud<PointT>::Ptr control_points(new pcl::PointCloud<PointT>);
    int num_poly = map_polygons.size();
    for (int i = 0; i < num_poly; ++i) {
        int num_pts = map_polygons[i].points.size();
        for (int j = 0; j < num_pts - 1; ++j) {
            std::ostringstream oss;
            oss << i << j << "_line";
            PointT s, t;
            s.x = map_polygons[i].points[j].x;
            s.y = map_polygons[i].points[j].y;
            s.z = 0;
            t.x = map_polygons[i].points[j + 1].x;
            t.y = map_polygons[i].points[j + 1].y;
            t.z = 0;
            pcl_vs->addLine<PointT, PointT>(s, t, 1.0, 0, 0, oss.str(), pcl_vp);
            control_points->points.push_back(s);
            control_points->points.push_back(t);
        }
    }
    add_pointcloud<PointT>(pcl_vs, control_points, 255, 0, 0, 4, "control_points", pcl_vp);
}

template <typename PointT>
void add_bbox_3d(Eigen::Vector3f& center, Eigen::Vector3f& size, 
        Eigen::Matrix3f& rot, Eigen::Vector3f& rgb, 
        pcl::visualization::PCLVisualizer* p, int viewport, std::string id) {
    float half_wid = size[0] / 2.0;
    float half_len = size[1] / 2.0;
    typename pcl::PointCloud<PointT>::Ptr bbox(new pcl::PointCloud<PointT>);
    get_bbox_polygon<PointT>(bbox, rot, center, half_wid, half_len);

    std::ostringstream oss_bbox;
    oss_bbox << id << "_bbox";
    p->addPolygon<PointT>(bbox, rgb[0], rgb[1], rgb[2], oss_bbox.str(), viewport); 
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
             2, oss_bbox.str(), viewport); 

    float height = size[2];
    for (int i = 0; i < bbox->points.size(); ++i) {
        PointT vp = bbox->points[i];
        bbox->points[i].z += height;
        std::ostringstream oss_vet;
        oss_vet << id << "_bbox_vet_" << i;
        p->addLine<PointT, PointT>(vp, bbox->points[i], 
                rgb[0], rgb[1], rgb[2], oss_vet.str(), viewport);
        p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                                2, oss_vet.str(), viewport);
    } 

    std::ostringstream oss_bbox_up;
    oss_bbox_up << id << "_bbox_up";
    p->addPolygon<PointT>(bbox, rgb[0], rgb[1], rgb[2], oss_bbox_up.str(), viewport); 
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
             2, oss_bbox_up.str(), viewport); 
}

bool GetPointCloudFromFile(const std::string& pcd_file,
                           pcl_util::PointCloudPtr cloud);


} //namespace perception
} //namespace apollo
#endif //namespace MODULES_PERCEPTION_OBSTACLE_VISUALIZATION_PCL_VIS_UTIL_H
