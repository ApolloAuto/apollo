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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_LAPS_CHECKER_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_LAPS_CHECKER_H

#include <vector>
#include <map>
#include <memory>
#include <utility>
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"

namespace apollo {
namespace hdmap {

struct GridMeta {
  double alpha;
  std::vector<int> idxs;
};
typedef std::vector<GridMeta> Grid;

class LapsChecker  {
 public:
  explicit LapsChecker(
    const std::vector<FramePose>& poses,
    int laps_to_check,
    std::shared_ptr<JSonConf> sp_conf);
  ErrorCode check();
  double get_progress();
  double get_confidence();
  size_t get_lap();
  ErrorCode get_return_state();

 private:
  void do_check();
  int setup_grids_map();
  int check_laps();
  int check_params();
  int get_min_max();
  int do_setup_grids_map();
  double calc_alpha(int pose_index);
  int put_pose_to_grid(int pose_index, int grid_y, int grid_x);
  int put_pose_to_neighbor_grid(int pose_index);
  int get_passed_grid(
    int pose_index,
    std::vector<int> * sp_grid_x,
    std::vector<int> * sp_grid_y);
  double slope(double x1, double y1, double x2, double y2);
  int gather_timestamps(
    std::vector<double> * sp_stamps,
    double alpha, int center_x, int center_y);
  inline int set_progress(double p);
  // debug related
  int print_grid_map_to_file();

 public:
  const std::vector<FramePose>& _poses;
  double _maxx, _maxy, _minx, _miny;
  // std::shared_ptr<std::vector<std::vector<Grid>>> _grids_map;
  std::vector<std::vector<Grid>> _grids_map;
  bool finished;

 private:
  // std::shared_ptr<std::vector<double>> _confidence;
  std::vector<double> _confidence;
  double _progress;
  size_t _laps_to_check;
  size_t _possible_max_laps;
  size_t _lap;
  std::shared_ptr<JSonConf> _sp_conf;
  ErrorCode _return_state;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_LAPS_CHECKER_H
