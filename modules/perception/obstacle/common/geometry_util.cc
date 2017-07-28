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
#include "modules/perception/obstacle/common/geometry_util.h"

namespace apollo {
namespace perception {

using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;

void TransAffineToMatrix4(const Eigen::Vector3d& translation,
                          const Eigen::Vector4d& rotation,
                          Eigen::Matrix4d* trans_matrix) {
  const double t_x = translation(0);
  const double t_y = translation(1);
  const double t_z = translation(2);

  const double qx = rotation(0);
  const double qy = rotation(1);
  const double qz = rotation(2);
  const double qw = rotation(3);

  (*trans_matrix) << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
      2 * (qx * qz + qy * qw), t_x, 2 * (qx * qy + qz * qw),
      1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), t_y,
      2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
      1 - 2 * (qx * qx + qy * qy), t_z, 0, 0, 0, 1;
}

void TransformCloud(pcl_util::PointCloudPtr cloud, const std::vector<int>& indices,
                    pcl_util::PointDCloud* trans_cloud) {
  if (trans_cloud->size() != indices.size()) {
    trans_cloud->resize(indices.size());
  }
  for (size_t i = 0; i < indices.size(); ++i) {
    const Point& p = cloud->at(indices[i]);
    Eigen::Vector3d v(p.x, p.y, p.z);
    pcl_util::PointD& tp = trans_cloud->at(i);
    tp.x = v.x();
    tp.y = v.y();
    tp.z = v.z();
    tp.intensity = p.intensity;
  }
}

double vector_cos_theta_2d_xy(Eigen::Vector3f& v1, Eigen::Vector3f& v2) {                           
    double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());                              
    double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());                              
    double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);             
    return cos_theta;                                                                               
}                                                                                                   
                                                                                                     
double vector_cos_theta_2d_xy(Eigen::Vector4f& v1, Eigen::Vector4f& v2) {                           
    double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());                              
    double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());                              
    double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);             
    return cos_theta;                                                                               
}       

 float dist_xy_to_bbox(const Eigen::Vector3f& center, const Eigen::Vector3f& dir,                    
    const Eigen::Vector3f& size, const Eigen::Vector3f& point) {                                    
                                                                                                
    Eigen::Vector3f dir2d(dir[0], dir[1], 0);                                                       
    dir2d.normalize();                                                                              
    Eigen::Vector3f diff = point - center;                                                          
    float dx = std::max<float>(0.0f, fabs(dir2d.dot(diff)) - size[0] / 2);                          
                                                                                                
    Eigen::Vector3f odir(-dir2d[1], dir2d[0], 0);                                                   
    float dy = std::max<float>(0.0f, fabs(odir.dot(diff)) - size[1] / 2);                           
                                                                                                
    return sqrt(dx*dx + dy*dy);                                                                     
}     

void max_min_distance_xy_bbox_to_bbox(const Eigen::Vector3f& center0, const Eigen::Vector3f& dir0,  
    const Eigen::Vector3f& size0, const Eigen::Vector3f& center1,                                   
    const Eigen::Vector3f& dir1, const Eigen::Vector3f& size1,                                      
    float& max_dist, float& min_dist) {                                                             
                                                                                                
    max_dist = 0;                                                                                   
    min_dist = 9999999;                                                                             
    float cx[4] = {-1, 1, -1, 1};                                                                   
    float cy[4] = {-1, -1, 1, 1};                                                                   
    Eigen::Vector3f dir0_2d(dir0[0], dir0[1], 0);                                                   
    dir0_2d.normalize();                                                                            
    Eigen::Vector3f odir0(-dir0_2d[1], dir0_2d[0], 0);                                              
                                                                                                
    Eigen::Vector3f dir1_2d(dir1[0], dir1[1], 0);                                                   
    dir1_2d.normalize();                                                                            
    Eigen::Vector3f odir1(-dir1_2d[1], -dir1_2d[0], 0);                                             
    float hl = size1[0] / 2;                                                                        
    float hw = size1[1] / 2;                                                                        
    for (int i = 0; i < 4; i++) {                                                                   
        Eigen::Vector3f diff = center0 + dir0_2d * cx[i] * size0[0] / 2                             
            + odir0 * cy[i] * size0[1] / 2 - center1;                                               
        float dx = fabs(diff.dot(dir1_2d));                                                         
        float dist = 0;                                                                             
        if (dx > hl) {                                                                              
            dist = dx - hl;                                                                         
        }                                                                                           
                                                                                                
        float dy = fabs(diff.dot(odir1));                                                           
        if (dy > hw) {                                                                              
            if (dist < dy - hw) {                                                                   
                dist = dy - hw;                                                                     
            }                                                                                       
        }                                                                                           
                                                                                                
        if (max_dist < dist) {                                                                      
            max_dist = dist;                                                                        
        }                                                                                           
        if (min_dist > dist) {                                                                      
            min_dist = dist;                                                                        
        }                                                                                           
    }                                                                                               
}               

//compute the angle of two vector                                                                   
double vector_theta_2d_xy(Eigen::Vector3f& v1, Eigen::Vector3f& v2) {                               
    double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());                              
    double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());                              
    double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);             
    double sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);                         
    if (cos_theta > 1) {cos_theta = 1;}                                                             
    if (cos_theta < -1) {cos_theta = -1;}                                                           
    double theta = acos(cos_theta);                                                                 
    if (sin_theta < 0) {                                                                            
        theta = -theta;                                                                             
    }                                                                                               
    return theta;                                                                                   
}         

}  // namespace perception
}  // namespace apollo
