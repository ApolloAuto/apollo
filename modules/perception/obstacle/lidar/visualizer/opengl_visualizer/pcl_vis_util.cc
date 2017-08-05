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

#include <pcl/io/pcd_io.h>

#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/pcl_vis_util.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

void draw_boundingbox_3d(pcl::visualization::PCLVisualizer* p, int r, int g, int b, 
        int line_size, std::string id, int viewport, 
        Eigen::Vector3f& min_pt, Eigen::Vector3f& max_pt) {

    pcl::ModelCoefficients coeffs;
    // translation
    coeffs.values.push_back((min_pt[0] + max_pt[0]) / 2.0);
    coeffs.values.push_back((min_pt[1] + max_pt[1]) / 2.0);
    coeffs.values.push_back((min_pt[2] + max_pt[2]) / 2.0);
    // rotation
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(1.0);
    // size
    coeffs.values.push_back(max_pt[0] - min_pt[0]);
    coeffs.values.push_back(max_pt[1] - min_pt[1]);
    coeffs.values.push_back(max_pt[2] - min_pt[2]);

    p->removeShape(id);
    p->addCube(coeffs, id);
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
            r / 255.0, g / 255.0, b / 255.0, id);
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_size, id);
}  

void draw_boundingbox_3d(pcl::visualization::PCLVisualizer* p, int r, int g, int b, 
        int line_size, std::string id, int viewport, Eigen::Vector3f &min_pt, 
        Eigen::Vector3f& max_pt, Eigen::Quaterniond& quat) {

    pcl::ModelCoefficients coeffs;

    // translation
    coeffs.values.push_back((min_pt[0] + max_pt[0]) / 2.0);
    coeffs.values.push_back((min_pt[1] + max_pt[1]) / 2.0);
    coeffs.values.push_back((min_pt[2] + max_pt[2]) / 2.0);

    // rotation
    coeffs.values.push_back(quat.w());
    coeffs.values.push_back(quat.x());
    coeffs.values.push_back(quat.y());
    coeffs.values.push_back(quat.z());

    // size
    coeffs.values.push_back(max_pt[0] - min_pt[0]);
    coeffs.values.push_back(max_pt[1] - min_pt[1]);
    coeffs.values.push_back(max_pt[2] - min_pt[2]);

    p->removeShape(id);
    p->addCube(coeffs, id);
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
            r / 255.0, g / 255.0, b / 255.0, id);
    p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_size, id);
}

Eigen::Vector3f get_cls_color(int cls) {
    Eigen::Vector3f rgb;
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

Eigen::Vector3f get_track_color(int track_id){
    int cid = track_id % 124 + 1;
    int r = cid % 5;
    int g = ((cid - r) / 5) % 5;
    int b = (cid - r - 5 * g) / 25;
    float id2intesity[5] = {0, 128, 255, 64, 192};
    
    return Eigen::Vector3f(id2intesity[r], id2intesity[g], id2intesity[b]);
}

bool GetPointCloudFromFile(const std::string& pcd_file,
                           pcl_util::PointCloudPtr cloud)
                            {
  pcl::PointCloud<pcl_util::PointXYZIT> ori_cloud;
  if (pcl::io::loadPCDFile(pcd_file, ori_cloud) < 0) {
    AERROR << "Failed to load pcd file: " << pcd_file;
    return false;
  }

  cloud->points.reserve(ori_cloud.points.size());
  for (size_t i = 0; i < ori_cloud.points.size(); ++i) {
    apollo::perception::pcl_util::Point point;
    point.x = ori_cloud.points[i].x;
    point.y = ori_cloud.points[i].y;
    point.z = ori_cloud.points[i].z;
    point.intensity = ori_cloud.points[i].intensity;
    if (isnan(ori_cloud.points[i].x)) {
      continue;
    }
    cloud->push_back(point);
  }

  return true;
}


}// namespace perception
}// namespace apollo
