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

#include "modules/localization/msf/local_tool/local_visualization/engine/visualization_engine.h"

#include <boost/filesystem.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

using cyber::common::DirectoryExists;
using cyber::common::EnsureDirectory;

#define PI 3.1415926535897932346

unsigned char color_table[3][3] = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}};

const char car_img_path[3][1024] = {
    "modules/localization/msf/local_tool/local_visualization/img/red_car.png",
    "modules/localization/msf/local_tool/local_visualization/img/green_car.png",
    "modules/localization/msf/local_tool/local_visualization/img/blue_car.png"};

// =================VisualizationEngine=================
bool MapImageKey::operator<(const MapImageKey &key) const {
  // Compare elements by priority.
  return std::forward_as_tuple(level, zone_id, node_north_id, node_east_id) <
         std::forward_as_tuple(key.level, key.zone_id, key.node_north_id,
                               key.node_east_id);
}

// =================MapImageCache=================
bool MapImageCache::Get(const MapImageKey &key, cv::Mat *image) {
  auto found_iter = _map.find(key);
  if (found_iter == _map.end()) {
    return false;
  }
  // move the corresponding key to list front
  _list.splice(_list.begin(), _list, found_iter->second);
  *image = found_iter->second->second;
  return true;
}

void MapImageCache::Set(const MapImageKey &key, const cv::Mat &image) {
  auto found_iter = _map.find(key);
  if (found_iter != _map.end()) {
    // move the corresponding key to list front
    _list.splice(_list.begin(), _list, found_iter->second);
    found_iter->second->second = image;
    return;
  }
  // reached capacity, remove node in list, remove key in map
  if (_map.size() == _capacity) {
    MapImageKey key_to_del = _list.back().first;
    _list.back().second.release();
    _list.pop_back();
    _map.erase(key_to_del);
  }
  _list.emplace_front(key, image);
  _map[key] = _list.begin();
}

// =================VisualizationEngine=================
VisualizationEngine::VisualizationEngine()
    : map_image_cache_(20),
      image_window_(1024, 1024, CV_8UC3, cv::Scalar(0, 0, 0)),
      big_window_(3072, 3072, CV_8UC3),
      tips_window_(48, 1024, CV_8UC3, cv::Scalar(0, 0, 0)) {}

bool VisualizationEngine::Init(const std::string &map_folder,
                               const std::string &map_visual_folder,
                               const VisualMapParam &map_param,
                               const unsigned int resolution_id,
                               const int zone_id,
                               const Eigen::Affine3d &extrinsic,
                               const unsigned int loc_info_num) {
  map_folder_ = map_folder;
  map_visual_folder_ = map_visual_folder;
  map_param_ = map_param;
  velodyne_extrinsic_ = extrinsic;
  loc_info_num_ = loc_info_num;
  expected_car_loc_id_ = loc_info_num;

  trajectory_groups_.resize(loc_info_num_);

  for (unsigned int i = 0; i < loc_info_num_; i++) {
    cv::Mat tem_mat = cv::imread(car_img_path[i % 3]);
    if (tem_mat.empty()) {
      is_draw_car_ = false;
      break;
    }
    car_img_mats_.push_back(tem_mat);
  }

  if (resolution_id_ >= map_param_.map_resolutions.size()) {
    AERROR << "Invalid resolution id.";
    return false;
  }

  zone_id_ = zone_id;
  resolution_id_ = resolution_id;

  cloud_img_ =
      cv::Mat(cv::Size(map_param_.map_node_size_x, map_param_.map_node_size_y),
              CV_8UC3);
  cloud_img_mask_ =
      cv::Mat(cv::Size(map_param_.map_node_size_x, map_param_.map_node_size_y),
              CV_8UC1);

  Preprocess(map_folder, map_visual_folder);

  std::string params_file = image_visual_resolution_path_ + "/param.txt";
  bool success = InitOtherParams(params_file);
  if (!success) {
    AERROR << "Init other params failed.";
  }

  cv::namedWindow(window_name_, CV_WINDOW_NORMAL);
  cv::resizeWindow(window_name_, 1024, 1024);

  is_init_ = true;

  return true;
}

void VisualizationEngine::Visualize(
    const std::vector<LocalizatonInfo> &loc_infos,
    const std::vector<Eigen::Vector3d> &cloud) {
  if (!is_init_) {
    AERROR << "Visualziation should be init first.";
    return;
  }

  if (loc_infos.size() != loc_info_num_) {
    AERROR << "Please check the localization info num.";
    return;
  }

  cur_loc_infos_ = loc_infos;

  if (!UpdateCarLocId(expected_car_loc_id_)) {
    if (!UpdateCarLocId(car_loc_id_)) {
      if (!UpdateCarLocId()) {
        return;
      }
    } else {
      if (expected_car_loc_id_ == loc_info_num_) {
        expected_car_loc_id_ = car_loc_id_;
      }
    }
  }

  UpdateTrajectoryGroups();
  cloud_ = cloud;
  Draw();
}

void VisualizationEngine::SetAutoPlay(bool auto_play) {
  auto_play_ = auto_play;
}

void VisualizationEngine::Preprocess(const std::string &map_folder,
                                     const std::string &map_visual_folder) {
  std::string image_path = map_folder_ + "/image";
  std::string image_visual_path = map_visual_folder;
  char buf[256];
  snprintf(buf, sizeof(buf), "/%03u", resolution_id_);
  image_visual_resolution_path_ = image_visual_path + buf;
  AINFO << "image_visual_resolution_path: " << image_visual_resolution_path_;
  std::string image_resolution_path = image_path + buf;
  AINFO << "image_resolution_path: " << image_resolution_path;

  if (!EnsureDirectory(image_visual_path)) {
    AERROR << "image_visual_path: " << image_visual_path
           << " cannot be created.";
    return;
  }

  if (DirectoryExists(image_visual_resolution_path_)) {
    AINFO << "image_visual_resolution_path: " << image_visual_resolution_path_
          << "already exists.";
    return;
  }
  if (!EnsureDirectory(image_visual_resolution_path_)) {
    AERROR << "image_visual_resolution_path: " << image_visual_resolution_path_
           << " cannot be created.";
    return;
  }

  boost::filesystem::path image_resolution_path_boost(image_resolution_path);
  // push path of map's images to vector
  std::vector<std::string> map_bin_path;
  boost::filesystem::recursive_directory_iterator end_iter;
  boost::filesystem::recursive_directory_iterator iter(
      image_resolution_path_boost);
  for (; iter != end_iter; ++iter) {
    if (!boost::filesystem::is_directory(*iter)) {
      if (iter->path().extension() == ".png") {
        map_bin_path.push_back(iter->path().string());
      }
    } else {
      std::string tmp = iter->path().string();
      tmp = tmp.substr(image_resolution_path.length(), tmp.length());
      tmp = image_visual_resolution_path_ + tmp;
      EnsureDirectory(tmp);
    }
  }

  if (map_bin_path.empty()) {
    return;
  }

  GenerateMutiResolutionImages(map_bin_path,
                               static_cast<int>(image_resolution_path.length()),
                               image_visual_resolution_path_);
}

void VisualizationEngine::Draw() {
  UpdateLevel();

  if (follow_car_) {
    SetViewCenter(car_pose_.translation()[0], car_pose_.translation()[1]);
  }

  MapImageKey iamge_key;
  CoordToImageKey(_view_center, &iamge_key);

  // get 3*3 images on that level
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      MapImageKey key = iamge_key;
      key.node_north_id += i * cur_stride_;
      key.node_east_id += j * cur_stride_;
      if (LoadImageToCache(key)) {
        map_image_cache_.Get(key, &subMat_[i + 1][j + 1]);
      } else {
        subMat_[i + 1][j + 1] =
            cv::Mat(1024, 1024, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      subMat_[i][j].copyTo(
          big_window_(cv::Rect(j * 1024, i * 1024, 1024, 1024)));
    }
  }

  cv::Point map_grid_index =
      CoordToMapGridIndex(_view_center, resolution_id_, cur_stride_);
  cv::Point node_grid_index = MapGridIndexToNodeGridIndex(map_grid_index);
  cv::Point bias = node_grid_index - map_grid_index;

  DrawTrajectory(bias);
  DrawCloud(bias);
  DrawLoc(bias);
  DrawStd(bias);

  int width = static_cast<int>(1024 * cur_scale_) / cur_stride_;
  int dis = width / 2;
  int left_top_x = node_grid_index.x + 1024 - dis;
  int left_top_y = node_grid_index.y + 1024 - dis;

  cv::resize(big_window_(cv::Rect(left_top_x, left_top_y, width, width)),
             image_window_, cv::Size(1024, 1024), 0, 0, CV_INTER_LINEAR);
  cv::flip(image_window_, image_window_, 0);

  DrawLegend();
  DrawInfo();
  DrawTips();

  cv::namedWindow(window_name_, CV_WINDOW_NORMAL);
  // cv::setMouseCallback(window_name_, processMouse, 0);
  cv::imshow(window_name_, image_window_);

  int waitTime = 0;
  if (auto_play_) {
    waitTime = 10;
  }
  ProcessKey(cv::waitKey(waitTime));
}

void VisualizationEngine::DrawTrajectory(const cv::Point &bias) {
  AINFO << "Draw trajectory.";
  if (cur_level_ == 0 && is_draw_trajectory_) {
    unsigned int i = (car_loc_id_ + 1) % loc_info_num_;
    for (unsigned int k = 0; k < loc_info_num_; k++) {
      std::map<double, Eigen::Vector2d> &trj = trajectory_groups_[i];
      if (trj.empty()) {
        i = (i + 1) % loc_info_num_;
        continue;
      }

      cv::Point lt;
      cv::Point pre_lt;
      std::map<double, Eigen::Vector2d>::iterator iter = trj.end();
      --iter;
      const Eigen::Vector2d &loc_2d = iter->second;
      lt = CoordToMapGridIndex(loc_2d, resolution_id_, cur_stride_);
      lt = lt + bias + cv::Point(1024, 1024);

      if (lt.x >= 0 && lt.y >= 0 && lt.x < 1024 * 3 && lt.y < 1024 * 3) {
        unsigned char b = color_table[i % 3][0];
        unsigned char g = color_table[i % 3][1];
        unsigned char r = color_table[i % 3][2];
        cv::circle(big_window_, lt, 4, cv::Scalar(b, g, r), 1);
        pre_lt = lt;

        int count = 0;
        while (iter != trj.begin() && count < 500) {
          --iter;
          const Eigen::Vector2d &loc_2d = iter->second;
          lt = CoordToMapGridIndex(loc_2d, resolution_id_, cur_stride_);
          lt = lt + bias + cv::Point(1024, 1024);

          if (lt.x >= 0 && lt.y >= 0 && lt.x < 1024 * 3 && lt.y < 1024 * 3) {
            unsigned char b = color_table[i % 3][0];
            unsigned char g = color_table[i % 3][1];
            unsigned char r = color_table[i % 3][2];

            cv::circle(big_window_, lt, 3, cv::Scalar(b, g, r), 1);
            cv::line(big_window_, pre_lt, lt, cv::Scalar(b, g, r), 1);
            pre_lt = lt;

            ++count;
          }
        }
      }

      i = (i + 1) % loc_info_num_;
    }
  }
}

void VisualizationEngine::DrawLoc(const cv::Point &bias) {
  AINFO << "Draw loc.";
  if (cur_level_ == 0) {
    unsigned int i = (car_loc_id_ + 1) % loc_info_num_;
    for (unsigned int k = 0; k < loc_info_num_; k++) {
      LocalizatonInfo &loc_info = cur_loc_infos_[i];
      cv::Point lt;
      const Eigen::Translation3d &loc = loc_info.location;

      Eigen::Vector2d loc_2d;
      loc_2d[0] = loc.x();
      loc_2d[1] = loc.y();

      lt = CoordToMapGridIndex(loc_2d, resolution_id_, cur_stride_);
      lt = lt + bias + cv::Point(1024, 1024);

      if (lt.x >= 0 && lt.y >= 0 && lt.x < 1024 * 3 && lt.y < 1024 * 3) {
        if (is_draw_car_ && loc_info.is_has_attitude) {
          const Eigen::Quaterniond &quatd = loc_info.attitude;
          double quaternion[] = {quatd.w(), quatd.x(), quatd.y(), quatd.z()};
          double euler_angle[3] = {0};
          QuaternionToEuler(quaternion, euler_angle);
          euler_angle[0] = euler_angle[0] * 180 / PI;
          euler_angle[1] = euler_angle[1] * 180 / PI;
          euler_angle[2] = euler_angle[2] * 180 / PI;
          // Eigen::Matrix3d matrix = quatd.toRotationMatrix();
          // Eigen::Vector3d euler_angle = matrix.eulerAngles(1, 0, 2);
          // euler_angle = euler_angle * 180 / PI;
          double yaw = euler_angle[2];

          cv::Mat mat_tem;
          cv::resize(car_img_mats_[i], mat_tem, cv::Size(48, 24), 0, 0,
                     CV_INTER_LINEAR);
          cv::Mat rotated_mat;
          // AINFO << "yaw: " << yaw;
          // RotateImg(mat_tem, rotated_mat, 90 - yaw);
          // RotateImg(mat_tem, rotated_mat, - yaw - 90);
          // RotateImg(mat_tem, rotated_mat, yaw + 90);
          RotateImg(mat_tem, &rotated_mat, yaw - 90);
          cv::Point car_lt =
              lt - cv::Point(rotated_mat.cols / 2, rotated_mat.rows / 2);
          cv::Point car_rb =
              car_lt + cv::Point(rotated_mat.cols, rotated_mat.rows);
          if (car_lt.x >= 0 && car_lt.y >= 0 && car_rb.x <= 1024 * 3 &&
              car_rb.y <= 1024 * 3) {
            cv::Mat mat_mask;
            cv::cvtColor(rotated_mat, mat_mask, CV_BGR2GRAY);
            rotated_mat.copyTo(
                big_window_(cv::Rect(car_lt.x, car_lt.y, rotated_mat.cols,
                                     rotated_mat.rows)),
                mat_mask);
          }
        } else {
          unsigned char b = color_table[i % 3][0];
          unsigned char g = color_table[i % 3][1];
          unsigned char r = color_table[i % 3][2];
          cv::circle(big_window_, lt, 4, cv::Scalar(b, g, r), 1);
        }
      }

      i = (i + 1) % loc_info_num_;
    }
  }
}

void VisualizationEngine::DrawStd(const cv::Point &bias) {
  AINFO << "Draw std.";
  if (cur_level_ == 0 && is_draw_std_) {
    unsigned int i = (car_loc_id_ + 1) % loc_info_num_;
    for (unsigned int k = 0; k < loc_info_num_; k++) {
      LocalizatonInfo &loc_info = cur_loc_infos_[i];
      if (loc_info.is_has_std) {
        cv::Point lt;
        const Eigen::Translation3d &loc = loc_info.location;
        const Eigen::Vector3d &std = loc_info.std_var;
        Eigen::Vector2d loc_2d;
        loc_2d[0] = loc.x();
        loc_2d[1] = loc.y();

        lt = CoordToMapGridIndex(loc_2d, resolution_id_, cur_stride_);
        lt = lt + bias + cv::Point(1024, 1024);

        if (lt.x >= 0 && lt.y >= 0 && lt.x < 1024 * 3 && lt.y < 1024 * 3) {
          unsigned char b = color_table[i % 3][0];
          unsigned char g = color_table[i % 3][1];
          unsigned char r = color_table[i % 3][2];

          cv::Size size(static_cast<int>(std::sqrt(std[0]) * 200.0 + 1.0),
                        static_cast<int>(std::sqrt(std[1]) * 200.0 + 1.0));
          cv::ellipse(big_window_, lt, size, 0, 0, 360, cv::Scalar(b, g, r), 2,
                      8);
        }
      }

      i = (i + 1) % loc_info_num_;
    }
  }
}

void VisualizationEngine::DrawCloud(const cv::Point &bias) {
  if (!cur_loc_infos_[car_loc_id_].is_has_attitude) {
    return;
  }

  AINFO << "Draw cloud.";
  if (cur_level_ == 0) {
    CloudToMat(car_pose_, velodyne_extrinsic_, cloud_, &cloud_img_,
               &cloud_img_mask_);
    cv::Point lt;
    lt = CoordToMapGridIndex(cloud_img_lt_coord_, resolution_id_, cur_stride_);
    lt = lt + bias + cv::Point(1024, 1024);

    cv::Point rb = lt + cv::Point(1024, 1024);
    if (lt.x >= 0 && lt.y >= 0 && rb.x <= 1024 * 3 && rb.y <= 1024 * 3) {
      cloud_img_.copyTo(big_window_(cv::Rect(lt.x, lt.y, 1024, 1024)),
                        cloud_img_mask_);
      // cloud_img_.copyTo(big_window_(cv::Rect(lt.x, lt.y, 1024, 1024)));
    }
  }
}

void VisualizationEngine::DrawLegend() {
  AINFO << "Draw legend.";
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.6;
  int thickness = 2.0;
  int baseline = 0;
  cv::Size textSize;

  for (unsigned int i = 0; i < loc_info_num_; i++) {
    LocalizatonInfo &loc_info = cur_loc_infos_[i];
    if (!loc_info.is_valid) {
      continue;
    }

    std::string text = loc_info.description;
    textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Point textOrg(790, (15 + textSize.height) * (i + 1));
    cv::putText(image_window_, text, textOrg, fontFace, fontScale,
                cv::Scalar(255, 0, 0), thickness, 8);

    unsigned char b = color_table[i % 3][0];
    unsigned char g = color_table[i % 3][1];
    unsigned char r = color_table[i % 3][2];
    cv::circle(
        image_window_,
        cv::Point(755, (15 + textSize.height) * (i + 1) - textSize.height / 2),
        8, cv::Scalar(b, g, r), 3);
  }
}

void VisualizationEngine::DrawInfo() {
  AINFO << "Draw info.";
  LocalizatonInfo &loc_info = cur_loc_infos_[car_loc_id_];

  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.8;
  int thickness = 2.0;
  int baseline = 0;
  cv::Size textSize;

  std::string text;

  // draw frame id.
  char info[256];
  snprintf(info, sizeof(info), "Frame: %d.", loc_info.frame_id);
  text = info;
  textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  cv::Point textOrg(10, 10 + textSize.height);
  cv::putText(image_window_, text, textOrg, fontFace, fontScale,
              cv::Scalar(255, 0, 0), thickness, 8);

  // draw timestamp.
  snprintf(info, sizeof(info), "Timestamp: %lf.", loc_info.timestamp);
  text = info;
  textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  textOrg = cv::Point(10, 2 * (10 + textSize.height));
  cv::putText(image_window_, text, textOrg, fontFace, fontScale,
              cv::Scalar(255, 0, 0), thickness, 8);
}

void VisualizationEngine::DrawTips() {
  AINFO << "Draw tips.";

  tips_window_.setTo(cv::Scalar(0, 0, 0));

  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.5;
  int thickness = 1.0;
  int baseline = 0;
  cv::Size textSize;

  std::string text;

  // draw tips.
  char info[256];
  snprintf(info, sizeof(info),
           "e: zoom out; q: zoom in; m: max scale; n: origin scale; w: up; s: "
           "down; a: move left; d: move right; r: move center");
  text = info;
  textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  cv::Point textOrg(5, 5 + textSize.height);
  cv::putText(tips_window_, text, textOrg, fontFace, fontScale,
              cv::Scalar(255, 255, 255), thickness, 8);

  snprintf(info, sizeof(info),
           "f: follow car/free view; p: play/pause; c: change current loc; 1: "
           "draw car/not; 2: draw trajectory/not; others: next");
  text = info;
  textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  textOrg = cv::Point(5, 2 * (5 + textSize.height));
  cv::putText(tips_window_, text, textOrg, fontFace, fontScale,
              cv::Scalar(255, 255, 255), thickness, 8);

  tips_window_.copyTo(image_window_(cv::Rect(0, 976, 1024, 48)));
}

void VisualizationEngine::UpdateLevel() {
  if (cur_scale_ > max_stride_ * 1.5) {
    SetScale(max_stride_ * 1.5);
  }
  if (cur_scale_ < 0.5) {
    SetScale(0.5);
  }
  // calculate which image level to use
  cur_level_ = 0;
  cur_stride_ = 1;
  double radius = cur_scale_ / 2;
  while (radius >= 1.0) {
    radius /= 2;
    ++cur_level_;
    cur_stride_ *= 2;
  }
}

void VisualizationEngine::GenerateMutiResolutionImages(
    const std::vector<std::string> &src_files, const int base_path_length,
    const std::string &dst_folder) {
  int x_min = INT_MAX;
  int x_max = -1;
  int y_min = INT_MAX;
  int y_max = -1;
  for (size_t i = 0; i < src_files.size(); ++i) {
    int len = static_cast<int>(src_files[i].length());
    // src_file example: image/000/north/50/00034661/00003386.png
    int y = std::stoi(src_files[i].substr(len - 21, 8));
    int x = std::stoi(src_files[i].substr(len - 12, 8));

    x_min = x < x_min ? x : x_min;
    x_max = x > x_max ? x : x_max;
    y_min = y < y_min ? y : y_min;
    y_max = y > y_max ? y : y_max;
  }
  y_max += 1;
  x_max += 1;

  // calculate how many level need to create
  int level = 1;
  int range = 1;
  while (range < x_max - x_min || range < y_max - y_min) {
    range *= 2;
    ++level;
  }

  // for image level 0, just copy from raw image
  for (unsigned int i = 0; i < src_files.size(); i++) {
    std::string p = src_files[i].substr(
        base_path_length, src_files[i].length() - 4 - base_path_length);
    p = dst_folder + p;
    cv::Mat tmp = cv::imread(src_files[i]);
    cv::imwrite(p + "_0.png", tmp);
  }

  // generate higher image level;
  std::string image_visual_path_dst = src_files[0].substr(
      base_path_length, src_files[0].length() - 22 - base_path_length);
  image_visual_path_dst = dst_folder + image_visual_path_dst;

  int pt_x = x_min;
  int pt_y = y_min;
  int step = 1;
  for (int lvl = 1; lvl < level; lvl++) {
    int nstep = 2 * step;
    for (pt_x = x_min; pt_x < x_max; pt_x += nstep) {
      for (pt_y = y_min; pt_y < y_max; pt_y += nstep) {
        bool flag = false;
        char ss[200];

        cv::Mat large(2048, 2048, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat small;

        for (int i = 0; i < 2; i++) {
          for (int j = 0; j < 2; j++) {
            snprintf(ss, sizeof(ss), "%s/%08d/%08d_%d.png",
                     image_visual_path_dst.c_str(), pt_y + i * step,
                     pt_x + j * step, lvl - 1);
            if (cyber::common::PathExists(ss)) {
              flag = true;
              cv::Mat img = cv::imread(ss);
              img.copyTo(large(cv::Rect(j * 1024, i * 1024, 1024, 1024)));
            }
          }
        }
        if (flag) {
          snprintf(ss, sizeof(ss), "%s/%08d/%08d_%d.png",
                   image_visual_path_dst.c_str(), pt_y, pt_x, lvl);
          cv::resize(large, small, cv::Size(1024, 1024), 0, 0, CV_INTER_LINEAR);
          cv::imwrite(ss, small);
        }
      }
    }
    step = nstep;
  }

  // write param
  std::fstream outf(dst_folder + "/param.txt", std::ios::out);
  outf << x_min << " " << y_min << std::endl;
  outf << x_max << " " << y_max << std::endl;
  outf << level << std::endl;
  outf << image_visual_path_dst.substr(
              dst_folder.length(),
              image_visual_path_dst.length() - dst_folder.length() - 3)
       << std::endl;
  outf.close();
}

bool VisualizationEngine::InitOtherParams(const std::string &params_file) {
  int x_min = 0;
  int y_min = 0;
  int x_max = 0;
  int y_max = 0;
  int level = 0;
  std::string path = "";

  std::ifstream inf(params_file);
  if (!inf.is_open()) {
    AERROR << "Open params file failed.";
    return false;
  }

  inf >> x_min >> y_min >> x_max >> y_max >> level >> path;
  inf.close();

  InitOtherParams(x_min, y_min, x_max, y_max, level, path);

  return true;
}

void VisualizationEngine::InitOtherParams(const int x_min, const int y_min,
                                          const int x_max, const int y_max,
                                          const int level,
                                          const std::string &path) {
  lt_node_index_.x = x_min;
  lt_node_index_.y = y_min;
  lt_node_grid_index_.x = lt_node_index_.x * map_param_.map_node_size_x;
  lt_node_grid_index_.y = lt_node_index_.y * map_param_.map_node_size_y;

  double init_center_x = (static_cast<double>(x_max + x_min) / 2 *
                          map_param_.map_resolutions[resolution_id_] *
                          map_param_.map_node_size_x) +
                         map_param_.map_min_x;
  double init_center_y = (static_cast<double>(y_max + y_min) / 2 *
                          map_param_.map_resolutions[resolution_id_] *
                          map_param_.map_node_size_y) +
                         map_param_.map_min_y;
  SetViewCenter(init_center_x, init_center_y);
  max_level_ = level;
  max_stride_ = 1;
  for (int i = 1; i < level; ++i) {
    max_stride_ *= 2;
  }
  // SetScale((double)max_stride_);
  image_visual_leaf_path_ = image_visual_resolution_path_ + path;
  AINFO << "image_visual_leaf_path: " << image_visual_leaf_path_;
}

void VisualizationEngine::CloudToMat(const Eigen::Affine3d &cur_pose,
                                     const Eigen::Affine3d &velodyne_extrinsic,
                                     const std::vector<Eigen::Vector3d> &cloud,
                                     cv::Mat *cloud_img,
                                     cv::Mat *cloud_img_mask) {
  unsigned int img_width = map_param_.map_node_size_x;
  unsigned int img_height = map_param_.map_node_size_y;
  Eigen::Vector3d cen = car_pose_.translation();
  cloud_img_lt_coord_[0] = cen[0] - map_param_.map_resolutions[resolution_id_] *
                                        (static_cast<float>(img_width) / 2.0f);
  cloud_img_lt_coord_[1] = cen[1] - map_param_.map_resolutions[resolution_id_] *
                                        (static_cast<float>(img_height) / 2.0f);

  cloud_img_.setTo(cv::Scalar(0, 0, 0));
  cloud_img_mask_.setTo(cv::Scalar(0));

  for (unsigned int i = 0; i < cloud.size(); i++) {
    const Eigen::Vector3d &pt = cloud[i];
    Eigen::Vector3d pt_global = cur_pose * velodyne_extrinsic * pt;

    int col = static_cast<int>((pt_global[0] - cloud_img_lt_coord_[0]) /
                               map_param_.map_resolutions[resolution_id_]);
    int row = static_cast<int>((pt_global[1] - cloud_img_lt_coord_[1]) /
                               map_param_.map_resolutions[resolution_id_]);
    if (col < 0 || row < 0 ||
        col >= static_cast<int>(map_param_.map_node_size_x) ||
        row >= static_cast<int>(map_param_.map_node_size_y)) {
      continue;
    }

    unsigned char b = color_table[car_loc_id_ % 3][0];
    unsigned char g = color_table[car_loc_id_ % 3][1];
    unsigned char r = color_table[car_loc_id_ % 3][2];
    cloud_img->at<cv::Vec3b>(row, col) = cv::Vec3b(b, g, r);
    cloud_img_mask->at<unsigned char>(row, col) = 1;
  }
}

void VisualizationEngine::CoordToImageKey(const Eigen::Vector2d &coord,
                                          MapImageKey *key) {
  key->level = cur_level_;

  DCHECK_LT(resolution_id_, map_param_.map_resolutions.size());
  key->zone_id = zone_id_;
  int n = static_cast<int>((coord[0] - map_param_.map_min_x) /
                           (static_cast<float>(map_param_.map_node_size_x) *
                            map_param_.map_resolutions[resolution_id_]));
  int m = static_cast<int>((coord[1] - map_param_.map_min_y) /
                           (static_cast<float>(map_param_.map_node_size_y) *
                            map_param_.map_resolutions[resolution_id_]));
  int max_n = static_cast<int>((map_param_.map_max_x - map_param_.map_min_x) /
                               (static_cast<float>(map_param_.map_node_size_x) *
                                map_param_.map_resolutions[resolution_id_]));
  int max_m = static_cast<unsigned int>(
      (map_param_.map_max_y - map_param_.map_min_y) /
      (static_cast<float>(map_param_.map_node_size_y) *
       map_param_.map_resolutions[resolution_id_]));

  if (n >= 0 && m >= 0 && n < max_n && m < max_m) {
    key->node_north_id = m;
    key->node_east_id = n;
  } else {
    DCHECK(false);  // should never reach here
  }

  m = static_cast<int>(key->node_north_id) - lt_node_index_.y;
  if (m < 0) {
    m = m - (cur_stride_ - 1);
  }
  key->node_north_id = m / cur_stride_ * cur_stride_ + lt_node_index_.y;

  n = static_cast<int>(key->node_east_id) - lt_node_index_.x;
  if (n < 0) {
    n = n - (cur_stride_ - 1);
  }
  key->node_east_id = n / cur_stride_ * cur_stride_ + lt_node_index_.x;
}

cv::Point VisualizationEngine::CoordToMapGridIndex(
    const Eigen::Vector2d &coord, const unsigned int resolution_id,
    const int stride) {
  cv::Point p;
  p.x = static_cast<int>((coord[0] - map_param_.map_min_x) /
                         map_param_.map_resolutions[resolution_id]);
  p.y = static_cast<int>((coord[1] - map_param_.map_min_y) /
                         map_param_.map_resolutions[resolution_id]);

  cv::Point pr;
  pr.x = p.x - lt_node_grid_index_.x;
  pr.y = p.y - lt_node_grid_index_.y;
  pr.x = pr.x / stride;
  pr.y = pr.y / stride;

  return pr;
}

cv::Point VisualizationEngine::MapGridIndexToNodeGridIndex(const cv::Point &p) {
  cv::Point pi;
  pi.x = p.x % map_param_.map_node_size_x;
  pi.x = pi.x < 0 ? pi.x + map_param_.map_node_size_x : pi.x;
  pi.y = p.y % map_param_.map_node_size_y;
  pi.y = pi.y < 0 ? pi.y + map_param_.map_node_size_y : pi.y;

  return pi;
}

bool VisualizationEngine::LoadImageToCache(const MapImageKey &key) {
  cv::Mat img;

  if (!map_image_cache_.Get(key, &img)) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%02d/%08d/%08d_%d.png",
             image_visual_leaf_path_.c_str(), key.zone_id, key.node_north_id,
             key.node_east_id, key.level);
    if (cyber::common::PathExists(path)) {
      img = cv::imread(path);
      AINFO << "visualizer load: " << path;
      map_image_cache_.Set(key, img);
      return true;
    } else {
      return false;
    }
  }
  return true;
}

void VisualizationEngine::RotateImg(const cv::Mat &in_img, cv::Mat *out_img,
                                    double angle) {
  int width = (in_img.cols > in_img.rows) ? in_img.cols : in_img.rows;
  width += 4;
  cv::Mat mat_tem(width, width, CV_8UC3);
  mat_tem.setTo(cv::Scalar(0, 0, 0));
  in_img.copyTo(
      mat_tem(cv::Rect(width / 2 - in_img.cols / 2, width / 2 - in_img.rows / 2,
                       in_img.cols, in_img.rows)));
  cv::Point2f pt(static_cast<float>(width) / 2.0f,
                 static_cast<float>(width) / 2.0f);
  cv::Mat rotation_mat = cv::getRotationMatrix2D(pt, angle, 1.0);
  cv::warpAffine(mat_tem, *out_img, rotation_mat, cv::Size(width, width));
}

void VisualizationEngine::SetViewCenter(const double center_x,
                                        const double center_y) {
  _view_center[0] = center_x;
  _view_center[1] = center_y;
}

void VisualizationEngine::UpdateViewCenter(const double move_x,
                                           const double move_y) {
  _view_center[0] += move_x;
  _view_center[1] += move_y;
}

void VisualizationEngine::SetScale(const double scale) { cur_scale_ = scale; }

void VisualizationEngine::UpdateScale(const double factor) {
  cur_scale_ *= factor;
}

bool VisualizationEngine::UpdateCarLocId() {
  for (unsigned int i = 0; i < loc_info_num_ - 1; i++) {
    unsigned int tem_car_loc_id = (car_loc_id_ + i + 1) % loc_info_num_;
    if (cur_loc_infos_[tem_car_loc_id].is_valid) {
      car_loc_id_ = tem_car_loc_id;
      car_pose_ = cur_loc_infos_[car_loc_id_].pose;
      return true;
    }
  }
  return false;
}

bool VisualizationEngine::UpdateCarLocId(const unsigned int car_loc_id) {
  if (car_loc_id >= loc_info_num_) {
    return false;
  }

  if (cur_loc_infos_[car_loc_id].is_valid) {
    car_loc_id_ = car_loc_id;
    car_pose_ = cur_loc_infos_[car_loc_id_].pose;
    return true;
  }

  return false;
}

bool VisualizationEngine::UpdateTrajectoryGroups() {
  for (unsigned int i = 0; i < loc_info_num_; i++) {
    if (cur_loc_infos_[i].is_valid) {
      Eigen::Vector2d loc;
      LocalizatonInfo &loc_info = cur_loc_infos_[i];
      loc[0] = loc_info.location.x();
      loc[1] = loc_info.location.y();
      std::map<double, Eigen::Vector2d> &trajectory = trajectory_groups_[i];
      trajectory[loc_info.timestamp] = loc;
    }
  }

  return true;
}

void VisualizationEngine::ProcessKey(int key) {
  const int move = 20;
  char c_key = static_cast<char>(key);
  switch (c_key) {
    case 'e': {
      UpdateScale(1.1);
      Draw();
      break;
    }
    case 'q': {
      UpdateScale(0.9);
      Draw();
      break;
    }
    case 'w': {
      UpdateViewCenter(0, move);
      Draw();
      break;
    }
    case 'a': {
      UpdateViewCenter(-move, 0);
      Draw();
      break;
    }
    case 's': {
      UpdateViewCenter(0, -move);
      Draw();
      break;
    }
    case 'd': {
      UpdateViewCenter(move, 0);
      Draw();
      break;
    }
    case 'n': {
      SetScale(1.0);
      Draw();
      break;
    }
    case 'm': {
      SetScale(max_stride_);
      Draw();
      break;
    }
    case 'r': {
      const Eigen::Vector3d &loc = car_pose_.translation();
      SetViewCenter(loc[0], loc[1]);
      Draw();
      break;
    }
    case 'f': {
      follow_car_ = !follow_car_;
      Draw();
      break;
    }
    case 'p': {
      auto_play_ = !auto_play_;
      break;
    }
    case 'c': {
      UpdateCarLocId();
      expected_car_loc_id_ = car_loc_id_;
      Draw();
      break;
    }
    case '1': {
      is_draw_car_ = !is_draw_car_;
      break;
    }
    case '2': {
      is_draw_trajectory_ = !is_draw_trajectory_;
      break;
    }
    default:
      break;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
