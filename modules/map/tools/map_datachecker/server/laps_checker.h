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
#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"
#include "modules/map/tools/map_datachecker/server/common.h"

namespace apollo {
namespace hdmap {

struct GridMeta {
  double alpha;
  std::vector<int> idxs;
};
typedef std::vector<GridMeta> Grid;

class LapsChecker {
 public:
  LapsChecker(const std::vector<FramePose>& poses, int laps_to_check,
              std::shared_ptr<JsonConf> sp_conf);
  ErrorCode Check();
  double GetProgress() const;
  double GetConfidence();
  size_t GetLap() const;
  ErrorCode GetReturnState();

 private:
  void DoCheck();
  int SetupGridsMap();
  int CheckLaps();
  int CheckParams();
  int GetMinMax();
  int DoSetupGridsMap();
  double CalcAlpha(int pose_index);
  int PutPoseToGrid(int pose_index, int grid_y, int grid_x);
  int PutPoseToNeighborGrid(int pose_index);
  int GetPassedGrid(int pose_index, std::vector<int>* sp_grid_x,
                    std::vector<int>* sp_grid_y);
  double Slope(double x1, double y1, double x2, double y2);
  int GatherTimestamps(std::vector<double>* sp_stamps, double alpha,
                       int center_x, int center_y);
  inline int SetProgress(double p);

 public:
  const std::vector<FramePose>& poses_;
  double maxx_, maxy_, minx_, miny_;
  std::vector<std::vector<Grid>> grids_map_;
  bool finished_;

 private:
  std::vector<double> confidence_;
  double progress_;
  size_t laps_to_check_;
  size_t possible_max_laps_;
  size_t lap_;
  std::shared_ptr<JsonConf> sp_conf_;
  ErrorCode return_state_;
};

}  // namespace hdmap
}  // namespace apollo
