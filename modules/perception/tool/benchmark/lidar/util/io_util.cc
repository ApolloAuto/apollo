/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/tool/benchmark/lidar/util/io_util.h"

#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "pcl/io/pcd_io.h"

#include "modules/perception/tool/benchmark/lidar/util/geo_util.h"

namespace apollo {
namespace perception {
namespace benchmark {

static const std::map<std::string,
                      std::function<bool(const std::string&, PointCloudPtr)>>
    s_load_method = {
        {"xyzl", load_pcl_pcds_xyzl},
        {"xyzit", load_pcl_pcds_xyzit},
};

bool load_pcl_pcds(const std::string& filename, PointCloudPtr cloud_out,
                   const std::string& cloud_type) {
  auto iter = s_load_method.find(cloud_type);
  if (iter == s_load_method.end()) {
    return false;
  }
  return iter->second(filename, cloud_out);
}

bool load_pcl_pcds_xyzit(const std::string& filename, PointCloudPtr cloud_out) {
  PointXYZITCloud org_cloud;
  if (pcl::io::loadPCDFile(filename, org_cloud) < 0) {
    std::cerr << "failed to load pcd file: " << filename << std::endl;
    return false;
  }

  cloud_out->points.reserve(org_cloud.points.size());
  for (size_t i = 0; i < org_cloud.points.size(); ++i) {
    Point point;
    point.x = org_cloud.points[i].x;
    point.y = org_cloud.points[i].y;
    point.z = org_cloud.points[i].z;
    point.intensity = org_cloud.points[i].intensity;
    if (std::isnan(org_cloud.points[i].x)) {
      continue;
    }
    cloud_out->push_back(point);
  }
  return true;
}

bool load_pcl_pcds_xyzl(const std::string& filename, PointCloudPtr cloud_out) {
  PointXYZLCloud org_cloud;
  if (pcl::io::loadPCDFile(filename, org_cloud) < 0) {
    std::cerr << "failed to load pcd file: " << filename << std::endl;
    return false;
  }

  cloud_out->points.reserve(org_cloud.points.size());
  for (size_t i = 0; i < org_cloud.points.size(); ++i) {
    Point point;
    point.x = org_cloud.points[i].x;
    point.y = org_cloud.points[i].y;
    point.z = org_cloud.points[i].z;
    point.intensity = 255;
    point.label = org_cloud.points[i].label;
    if (std::isnan(org_cloud.points[i].x)) {
      continue;
    }
    cloud_out->push_back(point);
  }
  return true;
}

bool load_frame_objects(const std::string& filename,
                        const std::set<std::string>& black_list,
                        std::vector<ObjectPtr>* objects_out,
                        std::vector<PointCloud>* left_boundary,
                        std::vector<PointCloud>* right_boundary,
                        std::vector<PointCloud>* road_polygon,
                        std::vector<PointCloud>* left_lane_boundary,
                        std::vector<PointCloud>* right_lane_boundary,
                        PointCloud* cloud) {
  std::fstream fin(filename.c_str());
  if (!fin.is_open()) {
    std::cerr << "frame objects file " << filename << " is not exist!"
              << std::endl;
    return false;
  }
  int frame_id = -1;
  int object_number = -1;
  fin >> frame_id >> object_number;

  objects_out->clear();

  for (int i = 0; i < object_number; ++i) {
    ObjectPtr obj(new Object);
    std::string sensor_type;
    int object_id = -1;
    int track_id = -1;
    bool is_background = true;
    float confidence = 0.0;
    std::string type;
    float bbox_center_x = 0.0;
    float bbox_center_y = 0.0;
    float bbox_center_z = 0.0;
    float bbox_length = 0.0;
    float bbox_width = 0.0;
    float bbox_height = 0.0;
    float yaw = 0.0;
    float roll = 0.0;
    float pitch = 0.0;
    float truncated = 0.0;
    float occluded = 0.0;
    float velocity_x = 0.0;
    float velocity_y = 0.0;
    float velocity_z = 0.0;
    int point_number = -1;
    int indice_number = -1;

    fin >> sensor_type >> object_id >> track_id >> is_background >>
        confidence >> type >> bbox_center_x >> bbox_center_y >> bbox_center_z >>
        bbox_length >> bbox_width >> bbox_height >> yaw >> roll >> pitch >>
        truncated >> occluded >> velocity_x >> velocity_y >> velocity_z;

    fin >> point_number;
    obj->cloud.reset(new PointCloud);
    obj->cloud->points.resize(point_number);
    for (int j = 0; j < point_number; ++j) {
      Point pt;
      fin >> pt.x >> pt.y >> pt.z >> pt.intensity;
      obj->cloud->points[j] = pt;
    }

    fin >> indice_number;
    obj->indices.reset(new PointIndices);
    obj->polygon.points.clear();
    if (indice_number >= 0) {
      obj->indices->indices.resize(indice_number);
      for (int j = 0; j < indice_number; ++j) {
        int pt_indice = -1;
        fin >> pt_indice;
        obj->indices->indices[j] = pt_indice;
      }
    } else {
      // we hack here, if indices number is negative,
      // we save the polygon points (x, y, z)
      int polygon_points_number = -indice_number;
      obj->polygon.points.resize(polygon_points_number);
      for (int j = 0; j < polygon_points_number; ++j) {
        fin >> obj->polygon.points[j].x >> obj->polygon.points[j].y >>
            obj->polygon.points[j].z;
      }
      indice_number = 0;
    }

    if (point_number == 0 && indice_number == 0) {
      std::cerr << "illegal object with empty point cloud: " << filename
                << " object " << i << std::endl;
    } else if (point_number != 0 && indice_number != 0 &&
               point_number != indice_number) {
      std::cerr << "illegal object with different points & indices" << filename
                << " object " << i << std::endl;
      return false;
    }

    obj->id = object_id;
    obj->confidence = confidence;
    obj->direction = Eigen::Vector3d(cos(yaw), sin(yaw), 0);
    obj->yaw = yaw;
    obj->roll = roll;
    obj->pitch = pitch;
    obj->center = Eigen::Vector3d(bbox_center_x, bbox_center_y, bbox_center_z);
    obj->length = bbox_length;
    obj->width = bbox_width;
    obj->height = bbox_height;
    obj->truncated = truncated;
    obj->occluded = occluded;
    obj->type = translate_string_to_type(type);
    obj->is_background = is_background;
    obj->track_id = track_id;
    obj->velocity = Eigen::Vector3d(velocity_x, velocity_y, velocity_z);
    obj->sensor_type = translate_string_to_sensor_type(sensor_type);

    if (black_list.empty() || black_list.find(type) == black_list.end()) {
      objects_out->push_back(obj);
    } else {
      std::cerr << "Ignore object " << i
                << " since it is in black list, type = " << type << std::endl;
    }
  }

  auto load_polygon = [&](PointCloud* poly) {
    size_t size = 0;
    fin >> size;
    poly->resize(size);
    for (size_t i = 0; i < size; ++i) {
      fin >> poly->points[i].x >> poly->points[i].y >> poly->points[i].z;
    }
  };

  if (left_boundary != nullptr && right_boundary != nullptr &&
      road_polygon != nullptr) {
    size_t boundary_size = 0;
    fin >> boundary_size;
    if (!fin.eof()) {
      left_boundary->resize(boundary_size);
      right_boundary->resize(boundary_size);
      for (size_t i = 0; i < boundary_size; ++i) {
        load_polygon(&left_boundary->at(i));
        load_polygon(&right_boundary->at(i));
      }
      size_t polygon_size = 0;
      fin >> polygon_size;
      road_polygon->resize(polygon_size);
      for (size_t i = 0; i < polygon_size; ++i) {
        load_polygon(&road_polygon->at(i));
      }
    }

    size_t label_size = 0;
    fin >> label_size;
    if (!fin.eof() && cloud != nullptr) {
      if (label_size != cloud->size()) {
        std::cerr << "Label size " << label_size << " not equal to "
                  << "cloud size " << cloud->size() << std::endl;
        fin.close();
        return false;
      }
      for (size_t i = 0; i < label_size; ++i) {
        fin >> cloud->at(i).label;
      }
    }
  }

  if (left_lane_boundary != nullptr && right_lane_boundary != nullptr) {
    size_t boundary_size = 0;
    fin >> boundary_size;
    if (!fin.eof()) {
      left_lane_boundary->resize(boundary_size);
      right_lane_boundary->resize(boundary_size);
      for (size_t i = 0; i < boundary_size; ++i) {
        load_polygon(&left_lane_boundary->at(i));
        load_polygon(&right_lane_boundary->at(i));
      }
    }
  }

  fin.close();
  return true;
}

bool load_sensor2world_pose(const std::string& filename,
                            Eigen::Matrix4d* pose_out_pt) {
  Eigen::Matrix4d& pose_out = *pose_out_pt;
  std::ifstream ifs(filename.c_str());
  if (!ifs.is_open()) {
    std::cerr << "failed to open file " << filename << std::endl;
    return false;
  }
  char buffer[1024];
  ifs.getline(buffer, 1024);
  int id = 0;
  double time_samp = 0;
  double quat[4];
  double matrix3x3[9];
  sscanf(buffer, "%d %lf %lf %lf %lf %lf %lf %lf %lf", &id, &(time_samp),
         &(pose_out(0, 3)), &(pose_out(1, 3)), &(pose_out(2, 3)), &(quat[0]),
         &(quat[1]), &(quat[2]), &(quat[3]));
  quaternion_to_rotation_matrix<double>(quat, matrix3x3);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      pose_out(i, j) = matrix3x3[i * 3 + j];
    }
  }

  return true;
}

bool save_frame_objects(const std::string& filename,
                        const std::vector<ObjectPtr>& objects, int frame_id) {
  std::ofstream fout(filename.c_str());
  if (!fout.is_open()) {
    std::cout << "Failed to open " << filename << "\n";
    return false;
  }

  int object_number = static_cast<int>(objects.size());
  fout << frame_id << "  " << object_number << "\n";
  for (auto& object : objects) {
    fout << translate_sensor_type_to_string(object->sensor_type) << " "
         << object->id << " " << object->track_id << " "
         << object->is_background << " " << std::setprecision(8)
         << object->confidence << " " << translate_type_to_string(object->type)
         << " " << object->reserve << " " << object->center(0) << " "
         << object->center(1) << " " << object->center(2) << " "
         << object->length << " " << object->width << " " << object->height
         << " " << object->yaw << " " << object->roll << " " << object->pitch
         << " " << object->truncated << " " << object->occluded << " "
         << object->velocity(0) << " " << object->velocity(1) << " "
         << object->velocity(2) << " ";
    fout << object->cloud->size() << " ";
    for (auto& point : object->cloud->points) {
      fout << std::setprecision(8) << point.x << " " << point.y << " "
           << point.z << " " << point.intensity << " ";
    }
    fout << object->indices->indices.size() << " ";
    for (auto& id : object->indices->indices) {
      fout << id << " ";
    }
    fout << std::endl;
  }
  return true;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
