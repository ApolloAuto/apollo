/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/tools/offline/transform_server.h"

#include "cyber/common/log.h"
#include "modules/perception/common/camera/common/util.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace perception {
namespace camera {

bool TransformServer::Init(const std::vector<std::string> &camera_names,
                           const std::string &params_path) {
  const std::string params_dir = params_path;
  // 1. Init lidar height
  try {
    YAML::Node lidar_height =
        YAML::LoadFile(params_dir + "/" + "velodyne128_height.yaml");
    Eigen::Affine3d trans;
    trans.linear() = Eigen::Matrix3d::Identity();
    AINFO << trans.translation() << " "
          << lidar_height["vehicle"]["parameters"]["height"];
    trans.translation() << 0.0, 0.0,
        lidar_height["vehicle"]["parameters"]["height"].as<double>();
    AddTransform("velodyne128", "ground", trans);
  } catch (YAML::InvalidNode &in) {
    AERROR << "load velodyne128 extrisic file error"
           << " YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<float> &bc) {
    AERROR << "load velodyne128 extrisic file error, "
           << "YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load velodyne128 extrisic file "
           << " error, YAML exception:" << e.what();
    return false;
  }
  // 2. Init lidar and camera extrinsic
  std::vector<std::string> extrinsic_filelist;
  extrinsic_filelist.push_back(params_dir +
                               "/velodyne128_novatel_extrinsics.yaml");
  for (const auto &camera_name : camera_names) {
    extrinsic_filelist.push_back(params_dir + "/" + camera_name +
                                 "_extrinsics.yaml");
  }

  for (const auto &yaml_file : extrinsic_filelist) {
    try {
      YAML::Node node = YAML::LoadFile(yaml_file);
      if (node.IsNull()) {
        AINFO << "Load " << yaml_file << " failed! please check!";
        return false;
      }
      std::string child_frame_id = node["child_frame_id"].as<std::string>();
      std::string frame_id = node["header"]["frame_id"].as<std::string>();
      double q[4] = {node["transform"]["rotation"]["w"].as<double>(),
                     node["transform"]["rotation"]["x"].as<double>(),
                     node["transform"]["rotation"]["y"].as<double>(),
                     node["transform"]["rotation"]["z"].as<double>()};
      double t[3] = {node["transform"]["translation"]["x"].as<double>(),
                     node["transform"]["translation"]["y"].as<double>(),
                     node["transform"]["translation"]["z"].as<double>()};
      Eigen::Quaterniond qq(q[0], q[1], q[2], q[3]);
      Eigen::Affine3d trans;
      trans.linear() = qq.matrix();
      trans.translation() << t[0], t[1], t[2];
      if (!AddTransform(child_frame_id, frame_id, trans)) {
        AINFO << "failed to add transform from " << child_frame_id << " to "
              << frame_id << std::endl;
      }
    } catch (YAML::InvalidNode &in) {
      AERROR << "load camera extrisic file " << yaml_file
             << " with error, YAML::InvalidNode exception";
      return false;
    } catch (YAML::TypedBadConversion<double> &bc) {
      AERROR << "load camera extrisic file " << yaml_file
             << " with error, YAML::TypedBadConversion exception";
      return false;
    } catch (YAML::Exception &e) {
      AERROR << "load camera extrisic file " << yaml_file
             << " with error, YAML exception:" << e.what();
      return false;
    }
  }
  return true;
}

bool TransformServer::LoadFromFile(const std::string &tf_input,
                                   float frequency) {
  if (frequency <= 0) {
    AERROR << "Error frequency value:" << frequency;
    return false;
  }
  std::ifstream fin(tf_input);
  Transform tf;
  int64_t ts;
  while (fin >> ts) {
    tf.timestamp = static_cast<double>(ts) * 1e-9;
    fin >> tf.tx;
    fin >> tf.ty;
    fin >> tf.tz;
    fin >> tf.qx;
    fin >> tf.qy;
    fin >> tf.qz;
    fin >> tf.qw;
    tf_.push_back(tf);
  }
  fin.close();
  error_limit_ = 1 / frequency / 2.0f;
  AINFO << "Load tf successfully. count: " << tf_.size()
        << " error limit:" << error_limit_;
  return true;
}

bool TransformServer::QueryPos(double timestamp, Eigen::Affine3d *pose) {
  for (auto &&tf : tf_) {
    if (Equal(timestamp, tf.timestamp, error_limit_)) {
      Eigen::Quaterniond rotation(tf.qw, tf.qx, tf.qy, tf.qz);
      pose->linear() = rotation.matrix();
      pose->translation() << tf.tx, tf.ty, tf.tz;
      AINFO << "Get Pose:\n" << pose->matrix();
      return true;
    }
  }
  return false;
}

bool TransformServer::AddTransform(const std::string &child_frame_id,
                                   const std::string &frame_id,
                                   const Eigen::Affine3d &transform) {
  vertices_.insert(child_frame_id);
  vertices_.insert(frame_id);

  auto begin = edges_.lower_bound(child_frame_id);
  auto end = edges_.upper_bound(child_frame_id);

  for (auto iter = begin; iter != end; ++iter) {
    if (iter->second.frame_id == frame_id) {
      return false;
    }
  }

  Edge e;
  e.child_frame_id = child_frame_id;
  e.frame_id = frame_id;
  e.transform = transform;

  Edge e_inv;
  e_inv.child_frame_id = frame_id;
  e_inv.frame_id = child_frame_id;
  e_inv.transform = transform.inverse();
  ADEBUG << "Add transform between " << frame_id << " and " << child_frame_id;
  edges_.insert({child_frame_id, e});
  edges_.insert({frame_id, e_inv});

  return true;
}

bool TransformServer::QueryTransform(const std::string &child_frame_id,
                                     const std::string &frame_id,
                                     Eigen::Affine3d *transform) {
  *transform = Eigen::Affine3d::Identity();

  if (child_frame_id == frame_id) {
    return true;
  }

  // Vertices does not exist
  if (vertices_.find(child_frame_id) == vertices_.end() ||
      vertices_.find(frame_id) == vertices_.end()) {
    return false;
  }

  std::map<std::string, bool> visited;
  for (const auto &item : vertices_) {
    visited[item] = false;
  }

  return FindTransform(child_frame_id, frame_id, transform, &visited);
}

bool TransformServer::FindTransform(const std::string &child_frame_id,
                                    const std::string &frame_id,
                                    Eigen::Affine3d *transform,
                                    std::map<std::string, bool> *visited) {
  Eigen::Affine3d loc_transform = Eigen::Affine3d::Identity();

  auto begin = edges_.lower_bound(child_frame_id);
  auto end = edges_.upper_bound(child_frame_id);

  (*visited)[child_frame_id] = true;
  for (auto iter = begin; iter != end; ++iter) {
    auto &edge = iter->second;
    if ((*visited)[edge.frame_id]) {
      continue;
    }

    ADEBUG << "from " << edge.child_frame_id << " to " << edge.frame_id
           << std::endl;

    loc_transform = edge.transform * loc_transform;

    if (edge.frame_id == frame_id) {
      *transform = loc_transform;
      return true;
    }

    Eigen::Affine3d tr = Eigen::Affine3d::Identity();
    if (FindTransform(edge.frame_id, frame_id, &tr, visited)) {
      loc_transform = tr * loc_transform;
      *transform = loc_transform;
      return true;
    }

    loc_transform = edge.transform.inverse() * loc_transform;
  }
  return false;
}

void TransformServer::print() {
  for (auto item : edges_) {
    AINFO << "----------------" << std::endl;
    AINFO << item.first << std::endl;
    AINFO << "edge: " << std::endl;
    AINFO << "from " << item.second.child_frame_id << " to "
          << item.second.frame_id << std::endl;
    Eigen::Affine3d trans = item.second.transform;
    Eigen::Quaterniond quat(trans.linear());
    AINFO << "rot: " << quat.x() << " " << quat.y() << " " << quat.z() << " "
          << quat.w() << std::endl;
    AINFO << "trans: " << trans.translation()[0] << " "
          << trans.translation()[1] << " " << trans.translation()[2]
          << std::endl;
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
