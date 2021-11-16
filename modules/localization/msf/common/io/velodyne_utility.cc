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

#include "modules/localization/msf/common/io/velodyne_utility.h"

#include "pcl/io/pcd_io.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"

namespace apollo {
namespace localization {
namespace msf {
namespace velodyne {

void LoadPcds(const std::string& file_path, const unsigned int frame_index,
              const Eigen::Affine3d& pose, VelodyneFrame* velodyne_frame,
              bool is_global) {
  velodyne_frame->frame_index = frame_index;
  velodyne_frame->pose = pose;
  LoadPcds(file_path, frame_index, pose, &velodyne_frame->pt3ds,
           &velodyne_frame->intensities, is_global);
}

void LoadPcds(const std::string& file_path, const unsigned int frame_index,
              const Eigen::Affine3d& pose,
              ::apollo::common::EigenVector3dVec* pt3ds,
              std::vector<unsigned char>* intensities, bool is_global) {
  Eigen::Affine3d pose_inv = pose.inverse();
  pcl::PointCloud<PointXYZIT>::Ptr cloud(new pcl::PointCloud<PointXYZIT>);
  if (pcl::io::loadPCDFile(file_path, *cloud) >= 0) {
    if (cloud->height == 1 || cloud->width == 1) {
      AINFO << "Un-organized-point-cloud";
      for (unsigned int i = 0; i < cloud->size(); ++i) {
        Eigen::Vector3d pt3d;
        pt3d[0] = (*cloud)[i].x;
        pt3d[1] = (*cloud)[i].y;
        pt3d[2] = (*cloud)[i].z;

        if (pt3d[0] == pt3d[0] && pt3d[1] == pt3d[1] && pt3d[2] == pt3d[2]) {
          Eigen::Vector3d pt3d_local;
          if (is_global) {
            pt3d_local = pose_inv * pt3d;
          } else {
            pt3d_local = pt3d;
          }
          unsigned char intensity =
              static_cast<unsigned char>((*cloud)[i].intensity);
          pt3ds->push_back(pt3d_local);
          intensities->push_back(intensity);
        }
      }
    } else {
      for (unsigned int h = 0; h < cloud->height; ++h) {
        for (unsigned int w = 0; w < cloud->width; ++w) {
          double x = cloud->at(w, h).x;
          double y = cloud->at(w, h).y;
          double z = cloud->at(w, h).z;
          Eigen::Vector3d pt3d(x, y, z);
          if (pt3d[0] == pt3d[0] && pt3d[1] == pt3d[1] && pt3d[2] == pt3d[2]) {
            Eigen::Vector3d pt3d_local;
            if (is_global) {
              pt3d_local = pose_inv * pt3d;
            } else {
              pt3d_local = pt3d;
            }
            unsigned char intensity =
                static_cast<unsigned char>(cloud->at(w, h).intensity);
            pt3ds->push_back(pt3d_local);
            intensities->push_back(intensity);
          }
        }
      }
    }
  } else {
    AERROR << "Failed to load PCD file: " << file_path;
  }
}

void LoadPcdPoses(const std::string& file_path,
                  ::apollo::common::EigenAffine3dVec* poses,
                  std::vector<double>* timestamps) {
  std::vector<unsigned int> pcd_indices;
  LoadPcdPoses(file_path, poses, timestamps, &pcd_indices);
}

void LoadPcdPoses(const std::string& file_path,
                  ::apollo::common::EigenAffine3dVec* poses,
                  std::vector<double>* timestamps,
                  std::vector<unsigned int>* pcd_indices) {
  poses->clear();
  timestamps->clear();
  pcd_indices->clear();

  FILE* file = fopen(file_path.c_str(), "r");
  if (file) {
    unsigned int index;
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qr;
    static constexpr int kSize = 9;
    while (fscanf(file, "%u %lf %lf %lf %lf %lf %lf %lf %lf\n", &index,
                  &timestamp, &x, &y, &z, &qx, &qy, &qz, &qr) == kSize) {
      Eigen::Translation3d trans(Eigen::Vector3d(x, y, z));
      Eigen::Quaterniond quat(qr, qx, qy, qz);
      poses->push_back(trans * quat);
      timestamps->push_back(timestamp);
      pcd_indices->push_back(index);
    }
    fclose(file);
  } else {
    AERROR << "Can't open file to read: " << file_path;
  }
}

void LoadPosesAndStds(const std::string& file_path,
                      ::apollo::common::EigenAffine3dVec* poses,
                      ::apollo::common::EigenVector3dVec* stds,
                      std::vector<double>* timestamps) {
  poses->clear();
  stds->clear();
  timestamps->clear();

  FILE* file = fopen(file_path.c_str(), "r");
  if (file) {
    unsigned int index;
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qr;
    double std_x, std_y, std_z;
    static constexpr int kSize = 12;
    while (fscanf(file, "%u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                  &index, &timestamp, &x, &y, &z, &qx, &qy, &qz, &qr, &std_x,
                  &std_y, &std_z) == kSize) {
      Eigen::Translation3d trans(Eigen::Vector3d(x, y, z));
      Eigen::Quaterniond quat(qr, qx, qy, qz);
      poses->push_back(trans * quat);
      timestamps->push_back(timestamp);

      Eigen::Vector3d std;
      std << std_x, std_y, std_z;
      stds->push_back(std);
    }
    fclose(file);
  } else {
    AERROR << "Can't open file to read: " << file_path;
  }
}

bool LoadExtrinsic(const std::string& file_path, Eigen::Affine3d* extrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      extrinsic->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      extrinsic->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      extrinsic->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        extrinsic->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}

}  // namespace velodyne
}  // namespace msf
}  // namespace localization
}  // namespace apollo
