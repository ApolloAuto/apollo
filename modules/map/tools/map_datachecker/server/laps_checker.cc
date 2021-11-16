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
#include "modules/map/tools/map_datachecker/server/laps_checker.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace apollo {
namespace hdmap {

LapsChecker::LapsChecker(const std::vector<FramePose> &poses, int laps_to_check,
                         std::shared_ptr<JsonConf> sp_conf)
    : poses_(poses), sp_conf_(sp_conf) {
  laps_to_check_ = laps_to_check;
  maxx_ = 0.0;
  maxy_ = 0.0;
  minx_ = 0.0;
  miny_ = 0.0;
  possible_max_laps_ = (laps_to_check_ + 1) * 10;
  confidence_.resize(possible_max_laps_ + 1, 0.0);
  finished_ = false;
  return_state_ = ErrorCode::SUCCESS;
  AINFO << "instance has " << poses.size() << " poses";
  AINFO << "confidence size: " << possible_max_laps_ + 1;
}

int LapsChecker::SetProgress(double p) {
  progress_ = p;
  return 0;
}

double LapsChecker::GetProgress() const { return progress_; }

size_t LapsChecker::GetLap() const { return lap_; }

double LapsChecker::GetConfidence() {
  double res = 0.0;
  lap_ = laps_to_check_;
  for (size_t i = 0; i < confidence_.size(); ++i) {
    AINFO << "confidence[" << i << "]: " << confidence_[i];
  }
  AINFO << "laps to check: " << laps_to_check_;
  for (size_t i = laps_to_check_; i < confidence_.size(); ++i) {
    res += confidence_[i];
  }
  AINFO << "current confidence: " << res
        << ",confidence thresh:" << sp_conf_->laps_rate_thresh;
  if (res < sp_conf_->laps_rate_thresh) {
    if (confidence_.empty()) {
      AINFO << "some problems induce lap problem";
      return 0.0;
    }
    res = confidence_[0];
    lap_ = 0;
    for (size_t i = 1; i < confidence_.size(); ++i) {
      if (i == laps_to_check_) {
        continue;
      }
      if (confidence_[i] > res) {
        lap_ = i;
        res = confidence_[i];
      }
    }
  }
  return res;
}

ErrorCode LapsChecker::Check() {
  if (poses_.empty()) {
    return_state_ = ErrorCode::ERROR_VERIFY_NO_GNSSPOS;
    return return_state_;
  }
  DoCheck();
  finished_ = true;
  return return_state_;
}

void LapsChecker::DoCheck() {
  AINFO << "do_check";
  SetProgress(0.0);
  int ret = 0;
  AINFO << "check->check_params";
  ret = CheckParams();
  if (ret < 0) {
    AINFO << "check_params failed";
  }
  SetProgress(0.1);

  AINFO << "check->setup_grids_map";
  ret = SetupGridsMap();
  if (ret < 0) {
    AINFO << "setup_grids_map failed";
  }
  SetProgress(0.5);
  AINFO << "check->setup_grids_map done";

  AINFO << "check->check_laps";
  ret = CheckLaps();
  if (ret < 0) {
    AINFO << "check_laps failed";
  }
  SetProgress(1.0);
  AINFO << "check->check_laps done";

  AINFO << "do_check done";
}

int LapsChecker::CheckParams() {
  int n_pose = static_cast<int>(poses_.size());
  if (n_pose < sp_conf_->laps_frames_thresh) {
    return -1;
  }
  return 0;
}

int LapsChecker::SetupGridsMap() {
  AINFO << "setup_grids_map->get_min_max";
  GetMinMax();
  AINFO << "setup_grids_map->do_setup_grids_map";
  int ret = DoSetupGridsMap();
  if (ret < 0) {
    AINFO << "do_setup_grids_map failed";
    return -1;
  }
  AINFO << "setup_grids_map done";
  return 0;
}

int LapsChecker::CheckLaps() {
  int height = static_cast<int>(grids_map_.size());
  if (height <= 2 || height > 1000000) {
    AINFO << "grids_map_ size error. height = " << height;
    return -1;
  }

  int width = static_cast<int>(grids_map_[0].size());
  if (width <= 2 || width >= 1000000) {
    AINFO << "grids_map_ size error. width = " << width;
    return -1;
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Grid &grid = grids_map_[y][x];
      size_t t_size = grid.size();
      if (t_size == 0) {
        continue;
      }
      for (size_t i = 0; i < t_size; ++i) {
        std::vector<double> stamps;
        GatherTimestamps(&stamps, grid[i].alpha, x, y);
        if (stamps.empty()) {
          continue;
        }
        std::sort(stamps.begin(), stamps.end());
        double thresh_in_sec = sp_conf_->laps_time_err_thresh * 60;
        size_t segment = 1;
        for (size_t j = 1; j < stamps.size(); ++j) {
          if (stamps[j] - stamps[j - 1] > thresh_in_sec) {
            segment++;
          }
        }
        if (segment <= possible_max_laps_) {
          confidence_[segment] += 1;
        }
      }
    }
  }

  double all = 0.0;
  for (size_t i = 0; i < confidence_.size(); ++i) {
    all += confidence_[i];
  }
  if (all == 0.0) {
    AINFO << "there seems no poses";
    return -1;
  }
  AINFO << "confidence size: " << confidence_.size();
  for (size_t i = 0; i < confidence_.size(); ++i) {
    confidence_[i] /= all;
  }

  return 0;
}

int LapsChecker::GatherTimestamps(std::vector<double> *sp_stamps, double alpha,
                                  int center_x, int center_y) {
  int search_d = sp_conf_->laps_search_diameter;
  if ((search_d & 1) == 0) {
    AINFO << "laps_search_diameter should be an odd";
    return -1;
  }
  int search_r = (search_d >> 1);
  size_t height = grids_map_.size(), width = grids_map_[0].size();
  std::vector<double> &stamps = *sp_stamps;
  stamps.clear();
  const size_t start_y = std::max(0, center_y - search_r);
  const size_t end_y =
      std::min(static_cast<int>(height) - 1, center_y + search_r);
  const size_t start_x = std::max(0, center_x - search_r);
  const size_t end_x =
      std::min(static_cast<int>(width) - 1, center_x + search_r);
  for (size_t y = start_y; y <= end_y; y++) {
    for (size_t x = start_x; x <= end_x; x++) {
      Grid &grid = grids_map_[y][x];
      for (size_t i = 0; i < grid.size(); ++i) {
        if (std::abs(alpha - grid[i].alpha) < sp_conf_->laps_alpha_err_thresh) {
          std::vector<int> &idxs = grid[i].idxs;
          for (size_t j = 0; j < idxs.size(); ++j) {
            if (idxs[j] >= static_cast<int>(poses_.size())) {
              AINFO << "index error, index: " << idxs[j]
                    << ", pose size: " << poses_.size();
            } else {
              stamps.push_back(poses_[idxs[j]].time_stamp);
            }
          }
        }
      }
    }
  }

  return 0;
}

int LapsChecker::GetMinMax() {
  minx_ = std::numeric_limits<double>::max();
  miny_ = std::numeric_limits<double>::max();
  maxx_ = std::numeric_limits<double>::min();
  maxy_ = std::numeric_limits<double>::min();
  size_t size = poses_.size();
  AINFO << "get_min_max pose size: " << size;
  for (size_t i = 0; i < size; ++i) {
    double tx = poses_[i].tx, ty = poses_[i].ty;
    if (tx < minx_) {
      minx_ = tx;
    }
    if (tx > maxx_) {
      maxx_ = tx;
    }
    if (ty < miny_) {
      miny_ = ty;
    }
    if (ty > maxy_) {
      maxy_ = ty;
    }
  }
  return 0;
}

int LapsChecker::DoSetupGridsMap() {
  size_t width = size_t(maxx_ - minx_ + 1);
  size_t height = size_t(maxy_ - miny_ + 1);
  AINFO << "grid map width: " << width << ", height: " << height;
  size_t size = poses_.size();
  if (1 >= size || 0 == height || 0 == width || height > 1000000 ||
      width > 1000000) {
    AINFO << "pose size: " << size << ", height: " << height
          << ", width: " << width;
    AINFO << "pose size error or grid map size error";
    return -1;
  }

  grids_map_.resize(height);
  for (size_t i = 0; i < height; ++i) {
    grids_map_[i].resize(width);
  }
  // first pose can not be used
  for (size_t i = 1; i < size; ++i) {
    int x = static_cast<int>(poses_[i].tx - minx_);
    int y = static_cast<int>(poses_[i].ty - miny_);
    PutPoseToGrid(static_cast<int>(i), y, x);
    PutPoseToNeighborGrid(static_cast<int>(i));
  }
  return 0;
}

double LapsChecker::CalcAlpha(int pose_index) {
  double vecx = poses_[pose_index].tx - poses_[pose_index - 1].tx;
  double vecy = poses_[pose_index].ty - poses_[pose_index - 1].ty;
  double alpha = acos(vecx / sqrt(vecx * vecx + vecy * vecy)) * 180 / M_PI;
  if (alpha > 0) {
    return alpha;
  }
  return 360 + alpha;
}

int LapsChecker::PutPoseToGrid(int pose_index, int grid_y, int grid_x) {
  if (pose_index <= 0) {
    return 0;
  }
  double alpha = CalcAlpha(pose_index);
  if (std::isnan(alpha)) {
    AERROR << "ignore static pose " << pose_index;
    return 0;
  }

  Grid &grid = grids_map_[grid_y][grid_x];
  size_t t_size = grid.size();
  for (size_t j = 0; j < t_size; ++j) {
    if (std::abs(alpha - grid[j].alpha) < sp_conf_->laps_alpha_err_thresh) {
      grid[j].idxs.push_back(pose_index);
      grid[j].alpha = (grid[j].alpha + alpha) / 2;
      return 0;
    }
  }

  GridMeta gm;
  gm.alpha = alpha;
  gm.idxs = {pose_index};
  grid.push_back(gm);
  return 0;
}

int LapsChecker::PutPoseToNeighborGrid(int pose_index) {
  if (pose_index <= 0) {
    return 0;
  }
  std::vector<int> x, y;
  GetPassedGrid(pose_index, &x, &y);
  for (size_t i = 0; i < x.size(); ++i) {
    PutPoseToGrid(pose_index, y[i], x[i]);
  }
  return 0;
}

int LapsChecker::GetPassedGrid(int pose_index, std::vector<int> *sp_grid_x,
                               std::vector<int> *sp_grid_y) {
  if (pose_index <= 0) {
    return 0;
  }
  std::vector<int> &grid_x = *sp_grid_x;
  std::vector<int> &grid_y = *sp_grid_y;
  grid_x.clear();
  grid_y.clear();
  double x = poses_[pose_index].tx - minx_;
  double y = poses_[pose_index].ty - miny_;
  double last_x = poses_[pose_index - 1].tx - minx_;
  double last_y = poses_[pose_index - 1].ty - miny_;
  if (std::abs(x - last_x) < 1e-6) {  // current trace is vertical
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
    double k = Slope(last_x, last_y, x, y);
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

double LapsChecker::Slope(double x1, double y1, double x2, double y2) {
  if (std::abs(x1 - x2) < 1e-6) {
    return std::numeric_limits<double>::max();
  }
  if (std::abs(y1 - y2) < 1e-6) {
    return 0.0;
  }
  return (y2 - y1) / (x2 - x1);
}

ErrorCode LapsChecker::GetReturnState() { return return_state_; }

}  // namespace hdmap
}  // namespace apollo
