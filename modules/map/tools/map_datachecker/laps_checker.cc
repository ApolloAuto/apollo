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
#include "modules/map/tools/map_datachecker/laps_checker.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <limits>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <fstream>

namespace apollo {
namespace hdmap {

LapsChecker::LapsChecker(const std::vector<FramePose> &poses,
  int laps_to_check,
  std::shared_ptr<JSonConf> sp_conf)
  :_poses(poses), _sp_conf(sp_conf) {
  AINFO << "LapsChecker construct function";
  _laps_to_check = laps_to_check;
  AINFO << "instance has " << poses.size() << " poses";
  _maxx = _maxy = _minx = _miny = 0.0;
  _possible_max_laps = (_laps_to_check + 1) * 10;
  AINFO << "confidence size: " << _possible_max_laps + 1;
  _confidence.resize(_possible_max_laps + 1, 0.0);
  finished = false;
  _return_state = ErrorCode::SUCCESS;
}

int LapsChecker::set_progress(double p) {
  _progress = p;
  return 0;
}

double LapsChecker::get_progress() {
  return _progress;
}

size_t LapsChecker::get_lap() {
  return _lap;
}

double LapsChecker::get_confidence() {
  double res = 0.0;
  _lap = _laps_to_check;
  for (size_t i = _laps_to_check; i < _confidence.size(); i++) {
    res += _confidence[i];
  }
  AINFO << "res: " << res
      << ", conf.laps_rate_thresh:" << _sp_conf->laps_rate_thresh;
  if (res < _sp_conf->laps_rate_thresh) {
    if (_confidence.size() == 0) {
      AINFO << "some problems induce lap problem";
      return 0.0;
    }
    AINFO << "res < _conf.laps_rate_thresh" << _sp_conf->laps_rate_thresh;
    res = _confidence[0];
    _lap = 0;
    for (size_t i = 1; i < _confidence.size(); i++) {
       if (i == _laps_to_check) {
         continue;
       }
       if (_confidence[i] > res) {
         _lap = i;
         res = _confidence[i];
       }
     }
  }
  // for debug
  // for(size_t i = 0; i < _confidence.size(); i++) {
  //   AINFO << "(laps, conf): (" << i << ", " << _confidence[i] << ")";
  // }
  return res;
}

ErrorCode LapsChecker::check() {
  if (_poses.size() == 0) {
    _return_state = ErrorCode::ERROR_VERIFY_NO_GNSSPOS;
    return _return_state;
  }
  do_check();
  finished = true;
  return _return_state;
}

void LapsChecker::do_check() {
  AINFO << "do_check";
  set_progress(0.0);
  int ret = 0;
  AINFO << "check->check_params";
  ret = check_params();
  if (ret < 0) {
    AINFO << "check_params failed";
  }
  set_progress(0.1);

  AINFO << "check->setup_grids_map";
  ret = setup_grids_map();
  if (ret < 0) {
    AINFO << "setup_grids_map failed";
  }
  set_progress(0.5);
  AINFO << "check->setup_grids_map done";

  AINFO << "check->check_laps";
  ret = check_laps();
  if (ret < 0) {
    AINFO << "check_laps failed";
  }
  set_progress(1.0);
  AINFO << "check->check_laps done";

  AINFO << "do_check done";
}


int LapsChecker::check_params() {
  int n_pose = static_cast<int>(_poses.size());
  if (n_pose < _sp_conf->laps_frames_thresh) {
    return -1;
  }
  return 0;
}

int LapsChecker::setup_grids_map() {
  AINFO << "setup_grids_map->get_min_max";
  get_min_max();
  AINFO << "setup_grids_map->do_setup_grids_map";
  int ret = do_setup_grids_map();
  if (ret < 0) {
    AINFO << "do_setup_grids_map failed";
    return -1;
  }
  AINFO << "setup_grids_map done";
  return 0;
}

int LapsChecker::check_laps() {
  int height = static_cast<int>(_grids_map.size());
  if (height <= 2 || height > 1000000) {
    AINFO << "_grids_map size error. height = " << height;
    return -1;
  }

  int width = static_cast<int>(_grids_map[0].size());
  if (width <= 2 || width >= 1000000) {
    AINFO << "_grids_map size error. width = " << width;
    return -1;
  }
  // double all = 0, valid = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Grid &grid = _grids_map[y][x];
      size_t t_size = grid.size();
      if (t_size == 0) {
        continue;
      }
      for (size_t i = 0; i < t_size; i++) {
        std::vector<double> stamps;
        gather_timestamps(&stamps, grid[i].alpha, x, y);
        if (stamps.size() == 0) {
          continue;
        }
        std::sort(stamps.begin(), stamps.end());
        double thresh_in_sec = _sp_conf->laps_time_err_thresh * 60;
        size_t segment = 1;
        for (size_t j = 1; j < stamps.size(); j++) {
          if (stamps[j] - stamps[j - 1] > thresh_in_sec) {
            segment++;
          }
        }
        if (segment <= _possible_max_laps) {
          _confidence[segment] += 1;
        }
      }
    }
  }

  double all = 0.0;
  for (size_t i = 0; i < _confidence.size(); i++) {
    all += _confidence[i];
  }
  if (all == 0.0) {
    AINFO << "there seems no poses";
    return -1;
  }
  AINFO << "confidence size: " << _confidence.size();
  for (size_t i = 0; i < _confidence.size(); i++) {
    _confidence[i] /= all;
  }

  return 0;
}

int LapsChecker::gather_timestamps(
  std::vector<double> * sp_stamps,
  double alpha, int center_x, int center_y) {
  int search_d = _sp_conf->laps_search_diameter;
  if ((search_d & 1) == 0) {
    AINFO << "laps_search_diameter should be an odd";
    return -1;
  }
  int search_r = (search_d >> 1);
  size_t height = _grids_map.size(), width = _grids_map[0].size();
  std::vector<double> & stamps = *sp_stamps;
  stamps.clear();
  const size_t start_y = std::max(0, center_y - search_r);
  const size_t end_y =
    std::min(static_cast<int>(height) - 1, center_y + search_r);
  const size_t start_x = std::max(0, center_x - search_r);
  const size_t end_x =
    std::min(static_cast<int>(width) - 1, center_x + search_r);
  for (size_t y = start_y; y <= end_y; y++) {
    for (size_t x = start_x; x <= end_x; x++) {
      Grid &grid = _grids_map[y][x];
      for (size_t i = 0; i < grid.size(); i++) {
        if (std::abs(alpha - grid[i].alpha) <
          _sp_conf->laps_alpha_err_thresh) {
          std::vector<int> &idxs = grid[i].idxs;
          for (size_t j = 0; j < idxs.size(); j++) {
            if (idxs[j] >= static_cast<int>(_poses.size())) {
              AINFO << "index error, index: " << idxs[j]
                  << ", pose size: " << _poses.size();
            } else {
              stamps.push_back(_poses[idxs[j]].time_stamp);
            }
          }
        }
      }
    }
  }

  return 0;
}

int LapsChecker::get_min_max() {
  _minx = _miny = std::numeric_limits<double>::max();
  _maxx = _maxy = std::numeric_limits<double>::min();

  size_t size = _poses.size();
  AINFO << "get_min_max pose size: " << size;
  for (size_t i = 0; i < size; i++) {
    double tx = _poses[i].tx, ty = _poses[i].ty;
    if (tx < _minx)
      _minx = tx;
    if (tx > _maxx)
      _maxx = tx;
    if (ty < _miny)
      _miny = ty;
    if (ty > _maxy)
      _maxy = ty;
  }

  return 0;
}

int LapsChecker::do_setup_grids_map() {
  size_t width = size_t(_maxx - _minx + 1);
  size_t height = size_t(_maxy - _miny + 1);
  AINFO << "grid map width: " << width
      << ", height: " << height;
  size_t size = _poses.size();
  if (1 >= size || 0 == height || 0 == width ||
    height > 1000000 || width > 1000000) {
    AINFO << "pose size: " << size
        << ", height: " << height
        << ", width: " << width;
    AINFO << "pose size error or grid map size error";
    return -1;
  }

  _grids_map.resize(height);
  for (size_t i = 0; i < height; i++) {
    _grids_map[i].resize(width);
  }
  // first pose can not be used
  for (size_t i = 1; i < size; i++) {
    int x = static_cast<int>(_poses[i].tx - _minx);
    int y = static_cast<int>(_poses[i].ty - _miny);
    put_pose_to_grid(static_cast<int>(i), y, x);
    put_pose_to_neighbor_grid(static_cast<int>(i));
  }
  return 0;
}

double LapsChecker::calc_alpha(int pose_index) {
  double vecx = _poses[pose_index].tx - _poses[pose_index - 1].tx;
  double vecy = _poses[pose_index].ty - _poses[pose_index - 1].ty;
  double alpha = acos(vecx / sqrt(vecx * vecx + vecy * vecy)) * 180 / M_PI;
  if (alpha > 0) {
    return alpha;
  }
  return 360 + alpha;
}

int LapsChecker::put_pose_to_grid(int pose_index, int grid_y, int grid_x) {
  if (pose_index <= 0) {
    return 0;
  }
  double alpha = calc_alpha(pose_index);
  if (std::isnan(alpha)) {
    AERROR << "ignore static pose " << pose_index;
    return 0;
  }

  Grid &grid = _grids_map[grid_y][grid_x];
  size_t t_size = grid.size();
  for (size_t j = 0; j < t_size; j++) {
    if (std::abs(alpha - grid[j].alpha) <
      _sp_conf->laps_alpha_err_thresh) {
      grid[j].idxs.push_back(pose_index);
      grid[j].alpha = (grid[j].alpha + alpha) / 2;
      return 0;
    }
  }
  // 没有找到可以当前pose的gridmeta
  GridMeta gm;
  gm.alpha = alpha;
  gm.idxs = {pose_index};
  grid.push_back(gm);
  return 0;
}


int LapsChecker::put_pose_to_neighbor_grid(int pose_index) {
  if (pose_index <= 0) {
    return 0;
  }

  std::vector<int> x, y;
  get_passed_grid(pose_index, &x, &y);
  // AINFO << "pose " << pose_index << " has " << x.size() << " passed grid";
  for (size_t i = 0; i < x.size(); i++) {
    put_pose_to_grid(pose_index, y[i], x[i]);
  }
  return 0;
}

int LapsChecker::get_passed_grid(
  int pose_index,
  std::vector<int> * sp_grid_x,
  std::vector<int> * sp_grid_y) {
  if (pose_index <= 0) {
    return 0;
  }
  std::vector<int> & grid_x = *sp_grid_x;
  std::vector<int> & grid_y = *sp_grid_y;
  grid_x.clear();
  grid_y.clear();
  double x = _poses[pose_index].tx - _minx;
  double y = _poses[pose_index].ty - _miny;
  double last_x = _poses[pose_index - 1].tx - _minx;
  double last_y = _poses[pose_index - 1].ty - _miny;
  if (std::abs(x - last_x) < 1e-6) {  // current trace is verticalx
    int start_y = static_cast<int>(std::min(y, last_y)) + 1;
    int end_y = static_cast<int>(std::max(y, last_y));
    int start_x = static_cast<int>(x);
    while ((start_y++) < end_y) {
      grid_x.push_back(start_x);
      grid_y.push_back(start_y);
    }
  } else if (std::abs(y - last_y) < 1e-6) {  // current trace is horizontal
    int start_x = static_cast<int>(std::min(x, last_x));
    int end_x = static_cast<int>(std::max(x, last_x));
    int start_y = static_cast<int>(y);
    while ((++start_x) < end_x) {
      grid_x.push_back(start_x);
      grid_y.push_back(start_y);
    }
  } else {
    double k = slope(last_x, last_y, x, y);
    if (k > 99999999.0) {
      AERROR << "slope error";
      return -1;
    }
    int steps = static_cast<int>(std::abs(last_x - x));
    int step = 0;
    double xx = 0.0, yy = 0.0;
    if (x < last_x) {
      xx = x, yy = y;
    } else {
      xx = last_x, yy = last_y;
    }
    while ((step++) < steps) {
      xx = xx + 1;
      yy = yy + k;
      grid_x.push_back(static_cast<int>(xx));
      grid_y.push_back(static_cast<int>(yy));
    }
  }
  return 0;
}

double LapsChecker::slope(double x1, double y1, double x2, double y2) {
  if (std::abs(x1 - x2) < 1e-6) {
    return std::numeric_limits<double>::max();
  }
  if (std::abs(y1 - y2) < 1e-6) {
    return 0.0;
  }
  return (y2 - y1) / (x2 - x1);
}

ErrorCode LapsChecker::get_return_state() {
  return _return_state;
}

}  // namespace hdmap
}  // namespace apollo
