#include "modules/localization/msf/local_tool/local_visualization/visualization_engine.h"
#include <stdio.h>
#include <boost/filesystem.hpp>
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"

namespace apollo {
namespace localization {
namespace msf {

// =================VisualizationEngine=================
bool MapImageKey::operator<(const MapImageKey &key) const {
  if (this->level < key.level) {
    return true;
  }
  if (this->level > key.level) {
    return false;
  }

  if (this->zone_id < key.zone_id) {
    return true;
  }
  if (this->zone_id > key.zone_id) {
    return false;
  }

  if (this->node_north_id < key.node_north_id) {
    return true;
  }
  if (this->node_north_id > key.node_north_id) {
    return false;
  }

  if (this->node_east_id < key.node_east_id) {
    return true;
  }

  return false;
}

// =================MapImageCache=================
bool MapImageCache::Get(const MapImageKey &key, cv::Mat &image) {
  auto found_iter = _map.find(key);
  if (found_iter == _map.end()) {
    return false;
  }
  // move the corresponding key to list front
  _list.splice(_list.begin(), _list, found_iter->second);
  image = found_iter->second->second;
  return true;
}

void MapImageCache::Set(const MapImageKey &key, cv::Mat &image) {
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
    : _image_window(1024, 1024, CV_8UC3, cv::Scalar(0, 0, 0)),
      _map_image_cache(20),
      _big_window(3072, 3072, CV_8UC3),
      _map_config() {
  _is_init = false;
  _follow_car = true;
  _auto_play = false;
  _resolution_id = 0;
  _cur_scale = 1.0;
  _cur_level = 0;
  _cur_stride = 1;
  _max_level = 0;
  _max_stride = 1;
  _zone_id = 50;

  _image_visual_resolution_path = "";
  _image_visual_leaf_path = "";

  _window_name = "Local Visualizer";
}

VisualizationEngine::~VisualizationEngine() {}

bool VisualizationEngine::Init(const std::string &map_folder,
                               const BaseMapConfig &map_config,
                               const unsigned int resolution_id,
                               const int zone_id,
                               const Eigen::Affine3d &extrinsic) {
  _map_folder = map_folder;
  _map_config = map_config;
  _velodyne_extrinsic = extrinsic;

  if (_resolution_id >= _map_config._map_resolutions.size()) {
    std::cerr << "Invalid resolution id." << std::endl;
    return false;
  }

  _zone_id = zone_id;
  _resolution_id = resolution_id;

  _cloud_img = cv::Mat(
      cv::Size(_map_config._map_node_size_x, _map_config._map_node_size_y),
      CV_8UC3);
  _cloud_img_mask = cv::Mat(
      cv::Size(_map_config._map_node_size_x, _map_config._map_node_size_y),
      CV_8UC1);

  Preprocess(map_folder);

  std::string params_file = _image_visual_resolution_path + "/param.txt";
  bool success = InitOtherParams(params_file);
  if (!success) {
    std::cerr << "Init other params failed." << std::endl;
  }

  _is_init = true;

  return true;
}

void VisualizationEngine::Visualize(const Eigen::Affine3d &cur_pose,
                                    const std::vector<Eigen::Vector3d> &cloud) {
  if (!_is_init) {
    return;
  }
  _car_pose = cur_pose;

  unsigned int img_width = _map_config._map_node_size_x;
  unsigned int img_height = _map_config._map_node_size_y;
  Eigen::Vector3d cen = cur_pose.translation();
  _cloud_img_lt_coord[0] =
      cen[0] -
      _map_config._map_resolutions[_resolution_id] * (img_width / 2.0f);
  _cloud_img_lt_coord[1] =
      cen[1] -
      _map_config._map_resolutions[_resolution_id] * (img_height / 2.0f);

  _cloud_img.setTo(cv::Scalar(0, 0, 0));
  _cloud_img_mask.setTo(cv::Scalar(0));
  CloudToMat(_car_pose, _velodyne_extrinsic, cloud, _cloud_img,
             _cloud_img_mask);
  Draw();
}

void VisualizationEngine::Preprocess(const std::string &map_folder) {
  std::string image_path = _map_folder + "/image";
  std::string image_visual_path = _map_folder + "/map_visual";
  char buf[256];
  snprintf(buf, 256, "/%03u", _resolution_id);
  _image_visual_resolution_path = image_visual_path + buf;
  std::cout << "image_visual_resolution_path: " << _image_visual_resolution_path
            << std::endl;
  std::string image_resolution_path = image_path + buf;
  std::cout << "image_resolution_path: " << image_resolution_path << std::endl;

  boost::filesystem::path image_visual_path_boost(image_visual_path);
  if (!boost::filesystem::exists(image_visual_path_boost)) {
    boost::filesystem::create_directory(image_visual_path_boost);
  }

  boost::filesystem::path image_resolution_path_boost(image_resolution_path);
  boost::filesystem::path image_visual_resolution_path_boost(
      _image_visual_resolution_path);
  // check if folder image_visual has been created
  if (boost::filesystem::exists(_image_visual_resolution_path)) {
    return;
  } else {
    boost::filesystem::create_directory(image_visual_resolution_path_boost);
  }

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
      tmp = _image_visual_resolution_path + tmp;
      boost::filesystem::path p(tmp);
      if (!boost::filesystem::exists(p)) boost::filesystem::create_directory(p);
    }
  }

  if (map_bin_path.size() == 0) {
    return;
  }

  GenerateMutiResolutionImages(map_bin_path, image_resolution_path.length(),
                               _image_visual_resolution_path);
}

void VisualizationEngine::Draw() {
  UpdateLevel();

  if (_follow_car) {
    SetViewCenter(_car_pose.translation()[0], _car_pose.translation()[1]);
  }

  MapImageKey iamge_key;
  CoordToImageKey(_view_center, iamge_key);

  // get 3*3 images on that level
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      MapImageKey key = iamge_key;
      key.node_north_id += i * _cur_stride;
      key.node_east_id += j * _cur_stride;
      if (LoadImageToCache(key)) {
        _map_image_cache.Get(key, _subMat[i + 1][j + 1]);
      } else {
        _subMat[i + 1][j + 1] =
            cv::Mat(1024, 1024, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      _subMat[i][j].copyTo(
          _big_window(cv::Rect(j * 1024, i * 1024, 1024, 1024)));
    }
  }

  cv::Point map_grid_index =
      CoordToMapGridIndex(_view_center, _resolution_id, _cur_stride);
  cv::Point node_grid_index = MapGridIndexToNodeGridIndex(map_grid_index);
  cv::Point bias = node_grid_index - map_grid_index;

  DrawCar(bias);
  DrawCloud(bias);

  int width = (int)(1024 * _cur_scale) / _cur_stride;
  int dis = width / 2;
  int left_top_x = node_grid_index.x + 1024 - dis;
  int left_top_y = node_grid_index.y + 1024 - dis;

  cv::resize(_big_window(cv::Rect(left_top_x, left_top_y, width, width)),
             _image_window, cv::Size(1024, 1024), 0, 0, CV_INTER_LINEAR);
  cv::flip(_image_window, _image_window, 0);
  cv::namedWindow(_window_name, CV_WINDOW_NORMAL);
  // cv::setMouseCallback(_window_name, processMouse, 0);
  cv::imshow(_window_name, _image_window);

  int waitTime = 0;
  if (_auto_play) {
    waitTime = 10;
  }
  ProcessKey(cv::waitKey(waitTime));
}

void VisualizationEngine::DrawCar(const cv::Point &bias) {
  if (_cur_level == 0) {
    cv::Point lt;
    const Eigen::Vector3d &loc = _car_pose.translation();
    Eigen::Vector2d loc_2d;
    loc_2d[0] = loc[0];
    loc_2d[1] = loc[1];

    lt = CoordToMapGridIndex(loc_2d, _resolution_id, _cur_stride);
    lt = lt + bias + cv::Point(1024, 1024);

    cv::circle(_big_window, lt, 4, cv::Scalar(0, 0, 255), 1);
  }
}

void VisualizationEngine::DrawCloud(const cv::Point &bias) {
  if (_cur_level == 0) {
    cv::Point lt;
    lt = CoordToMapGridIndex(_cloud_img_lt_coord, _resolution_id, _cur_stride);
    lt = lt + bias + cv::Point(1024, 1024);

    cv::Point rb = lt + cv::Point(1024, 1024);
    if (lt.x >= 0 && lt.y >= 0 && rb.x <= 1024 * 3 && rb.y <= 1024 * 3) {
      _cloud_img.copyTo(_big_window(cv::Rect(lt.x, lt.y, 1024, 1024)),
                        _cloud_img_mask);
      // _cloud_img.copyTo(_big_window(cv::Rect(lt.x, lt.y, 1024, 1024)));
    }
  }
}

void VisualizationEngine::UpdateLevel() {
  if (_cur_scale > _max_stride * 1.5) SetScale(_max_stride * 1.5);
  if (_cur_scale < 0.5) SetScale(0.5);

  // caculate which image level to use
  _cur_level = 0;
  _cur_stride = 1;
  double radius = _cur_scale / 2;
  while (radius >= 1.0) {
    radius /= 2;
    ++_cur_level;
    _cur_stride *= 2;
  }
}

void VisualizationEngine::GenerateMutiResolutionImages(
    const std::vector<std::string> &src_files, const int base_path_length,
    const std::string &dst_folder) {
  int x_min = INT_MAX;
  int x_max = -1;
  int y_min = INT_MAX;
  int y_max = -1;
  for (unsigned int i = 0; i < src_files.size(); ++i) {
    int len = src_files[i].length();
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

  // caculate how many level need to create
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
    // std::cerr << p << std::endl;
  }

  // generate higher image level;
  std::string image_visual_path_dst = src_files[0].substr(
      base_path_length, src_files[0].length() - 22 - base_path_length);
  image_visual_path_dst = dst_folder + image_visual_path_dst;
  // std::cerr << image_visual_path_dst << std::endl;

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
            snprintf(ss, 200, "%s/%08d/%08d_%d.png",
                     image_visual_path_dst.c_str(), pt_y + i * step,
                     pt_x + j * step, lvl - 1);
            // std::cerr << ss << std::endl;
            if (boost::filesystem::exists(boost::filesystem::path(ss))) {
              flag = true;
              cv::Mat img = cv::imread(ss);
              img.copyTo(large(cv::Rect(j * 1024, i * 1024, 1024, 1024)));
            }
          }
        }
        if (flag) {
          snprintf(ss, 200, "%s/%08d/%08d_%d.png",
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
  return;
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
    std::cerr << "Open params file failed." << std::endl;
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
  _lt_node_index.x = x_min;
  _lt_node_index.y = y_min;
  _lt_node_grid_index.x = _lt_node_index.x * _map_config._map_node_size_x;
  _lt_node_grid_index.y = _lt_node_index.y * _map_config._map_node_size_y;

  double init_center_x = (double(x_max + x_min) / 2 *
                          _map_config._map_resolutions[_resolution_id] *
                          _map_config._map_node_size_x) +
                         _map_config._map_range.GetMinX();
  double init_center_y = (double(y_max + y_min) / 2 *
                          _map_config._map_resolutions[_resolution_id] *
                          _map_config._map_node_size_y) +
                         _map_config._map_range.GetMinY();
  SetViewCenter(init_center_x, init_center_y);
  _max_level = level;
  _max_stride = 1;
  for (int i = 1; i < level; ++i) {
    _max_stride *= 2;
  }
  // SetScale((double)_max_stride);
  _image_visual_leaf_path = _image_visual_resolution_path + path;
  std::cout << "image_visual_leaf_path: " << _image_visual_leaf_path
            << std::endl;
}

void VisualizationEngine::CloudToMat(const Eigen::Affine3d &cur_pose,
                                     const Eigen::Affine3d &velodyne_extrinsic,
                                     const std::vector<Eigen::Vector3d> &cloud,
                                     cv::Mat &cloud_img,
                                     cv::Mat &cloud_img_mask) {
  for (unsigned int i = 0; i < cloud.size(); i++) {
    const Eigen::Vector3d &pt = cloud[i];
    Eigen::Vector3d pt_global = cur_pose * velodyne_extrinsic * pt;

    int col = (pt_global[0] - _cloud_img_lt_coord[0]) /
              _map_config._map_resolutions[_resolution_id];
    int row = (pt_global[1] - _cloud_img_lt_coord[1]) /
              _map_config._map_resolutions[_resolution_id];
    if (col < 0 || row < 0 || col >= _map_config._map_node_size_x ||
        row >= _map_config._map_node_size_y) {
      continue;
    }

    cloud_img.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 255);
    cloud_img_mask.at<unsigned char>(row, col) = 1;
  }
}

void VisualizationEngine::CoordToImageKey(const Eigen::Vector2d &coord,
                                          MapImageKey &key) {
  key.level = _cur_level;
  // get MapNodeIndex at image level 0
  MapNodeIndex index = MapNodeIndex::get_map_node_index(
      _map_config, coord, _resolution_id, _zone_id);

  key.zone_id = _zone_id;
  key.node_north_id = index._m;
  key.node_east_id = index._n;

  int m = (int)key.node_north_id - _lt_node_index.y;
  if (m < 0) m = m - (_cur_stride - 1);
  key.node_north_id = m / _cur_stride * _cur_stride + _lt_node_index.y;

  int n = (int)key.node_east_id - _lt_node_index.x;
  if (n < 0) n = n - (_cur_stride - 1);
  key.node_east_id = n / _cur_stride * _cur_stride + _lt_node_index.x;
}

cv::Point VisualizationEngine::CoordToMapGridIndex(
    const Eigen::Vector2d &coord, const unsigned int resolution_id,
    const int stride) {
  cv::Point p;
  p.x = static_cast<int>((coord[0] - _map_config._map_range.GetMinX()) /
                         _map_config._map_resolutions[resolution_id]);
  p.y = static_cast<int>((coord[1] - _map_config._map_range.GetMinY()) /
                         _map_config._map_resolutions[resolution_id]);

  cv::Point pr;
  pr.x = p.x - _lt_node_grid_index.x;
  pr.y = p.y - _lt_node_grid_index.y;
  pr.x = pr.x / stride;
  pr.y = pr.y / stride;

  return pr;
}

cv::Point VisualizationEngine::MapGridIndexToNodeGridIndex(const cv::Point &p) {
  cv::Point pi;
  pi.x = p.x % _map_config._map_node_size_x;
  pi.x = pi.x < 0 ? pi.x + _map_config._map_node_size_x : pi.x;
  pi.y = p.y % _map_config._map_node_size_y;
  pi.y = pi.y < 0 ? pi.y + _map_config._map_node_size_y : pi.y;

  return pi;
}

bool VisualizationEngine::LoadImageToCache(const MapImageKey &key) {
  cv::Mat img;

  if (!_map_image_cache.Get(key, img)) {
    char path[1024];
    snprintf(path, 1024, "%s/%02d/%08d/%08d_%d.png",
             _image_visual_leaf_path.c_str(), key.zone_id, key.node_north_id,
             key.node_east_id, key.level);
    if (boost::filesystem::exists(boost::filesystem::path(path))) {
      img = cv::imread(path);
      std::cout << "visualizer load: " << path << std::endl;
      _map_image_cache.Set(key, img);
      return true;
    } else {
      return false;
    }
  }
  return true;
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

void VisualizationEngine::SetScale(const double scale) {
  _cur_scale = scale;
}

void VisualizationEngine::UpdateScale(const double factor) {
  _cur_scale *= factor;
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
      UpdateViewCenter(0, -move);
      Draw();
      break;
    }
    case 'a': {
      UpdateViewCenter(-move, 0);
      Draw();
      break;
    }
    case 's': {
      UpdateViewCenter(0, move);
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
      SetScale(_max_stride);
      Draw();
      break;
    }
    case 'r': {
      const Eigen::Vector3d &loc = _car_pose.translation();
      SetViewCenter(loc[0], loc[1]);
      Draw();
      break;
    }
    case 'f': {
      _follow_car = !_follow_car;
      Draw();
      break;
    }
    case 'p': {
      _auto_play = !_auto_play;
    }
    default:
      break;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
