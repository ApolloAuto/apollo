/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar/lib/segmentation/ncut/common/flood_fill.h"
#include <cfloat>
#include <ctime>
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace lidar {

namespace {
const int kNumDirections = 8;
const int di[kNumDirections] = {+1, 0, -1, 0, +1, +1, -1, -1};
const int dj[kNumDirections] = {0, +1, 0, -1, +1, -1, +1, -1};
const int kEmptyGridLabel = -1;
const int kNonEmptyGridLabel = -2;
}  // namespace

int FloodFill::Pos(float x, float y) const {
  const int irow = static_cast<int>((y + _offset_y) / _cell_size);
  if (!IsValidRowIndex(irow)) {
    return -1;
  }
  const int jcol = static_cast<int>((x + _offset_x) / _cell_size);
  if (!IsValidColIndex(jcol)) {
    return -1;
  }
  return irow * _grid_num_cols + jcol;
}

bool FloodFill::Pos2d(float x, float y, int* irow, int* jcol) const {
  *irow = static_cast<int>((y + _offset_y) / _cell_size);
  if (!IsValidRowIndex(*irow)) {
    return false;
  }
  *jcol = static_cast<int>((x + _offset_x) / _cell_size);
  if (!IsValidColIndex(*jcol)) {
    return false;
  }
  return true;
}

void FloodFill::BuildGrid(base::PointFCloudConstPtr cloud) {
  CHECK_GT(_grid_radius, 0.0);
  CHECK_GT(_cell_size, 0.0);
  // .1 calculate grid size
  //  max_grid_size = 2.f *_grid_radius /_cell_size;
  _num_points = static_cast<int>(cloud->size());
  const double min_grid_radius = _grid_radius / 10.0;
  float min_x = static_cast<float>(-min_grid_radius);
  float min_y = static_cast<float>(-min_grid_radius);
  float max_x = static_cast<float>(min_grid_radius);
  float max_y = static_cast<float>(min_grid_radius);

  for (size_t i = 0; i < cloud->size(); ++i) {
    auto pt = (*cloud)[i];
    max_x = std::max(max_x, pt.x);
    min_x = std::min(min_x, pt.x);
    max_y = std::max(max_y, pt.y);
    min_y = std::min(min_y, pt.y);
  }
  CHECK_LE(min_x, max_x);
  CHECK_LE(min_y, max_y);
  // .2 move origin to the left bottom corner
  const float lower_x = std::max(min_x, -_grid_radius);
  const float lower_y = std::max(min_y, -_grid_radius);
  const float upper_x = std::min(max_x, _grid_radius);
  const float upper_y = std::min(max_y, _grid_radius);
  _offset_x = -lower_x;
  _offset_y = -lower_y;
  _grid_num_rows = static_cast<int>(ceil((upper_y - lower_y) / _cell_size)) + 1;
  _grid_num_cols = static_cast<int>(ceil((upper_x - lower_x) / _cell_size)) + 1;
  CHECK_GT(_grid_num_rows, 0);
  CHECK_GT(_grid_num_cols, 0);
  _grid_size = _grid_num_rows * _grid_num_cols;
  // .3 locate points into grid
  _point_cloud_grid_idx.assign(_num_points, -1);
  _label.assign(_grid_size, kEmptyGridLabel);
  for (int i = 0; i < _num_points; ++i) {
    const int idx = Pos((*cloud)[i].x, (*cloud)[i].y);
    if (idx >= 0) {
      _point_cloud_grid_idx[i] = idx;
      _label[idx] = kNonEmptyGridLabel;
    }
  }
}

int FloodFill::GetConnectedComponents() {
  int num_components = 0;
  for (int idx = 0; idx < _grid_size; ++idx) {
    auto& label = _label[idx];
    if (label == kNonEmptyGridLabel) {
      label = num_components;
      DfsColoring(idx / _grid_num_cols, idx % _grid_num_cols, num_components);
      ++num_components;
    }
  }
  return num_components;
}

void FloodFill::DfsColoring(int i, int j, int curr_component) {
  // recursively label the neighbors
  for (int direction = 0; direction < kNumDirections; ++direction) {
    const int i2 = i + di[direction];
    const int j2 = j + dj[direction];
    if (IsValidRowIndex(i2) && IsValidColIndex(j2)) {
      auto& label = _label[i2 * _grid_num_cols + j2];
      if (label == kNonEmptyGridLabel) {
        label = curr_component;
        DfsColoring(i2, j2, curr_component);
      }
    }
  }
}

void FloodFill::GetSegments(base::PointFCloudConstPtr cloud,
                            std::vector<std::vector<int> >* segments_indices,
                            std::vector<int>* num_cells_per_segment) {
  CHECK_NOTNULL(segments_indices);
  CHECK_NOTNULL(num_cells_per_segment);
  // .1 build grid
  BuildGrid(cloud);
  // .2 get connected components
  const int num_segments = GetConnectedComponents();
  CHECK_GE(num_segments, 0);
  // .3 write result
  segments_indices->clear();
  segments_indices->resize(num_segments);
  num_cells_per_segment->clear();
  num_cells_per_segment->resize(num_segments, 0);
  for (int i = 0; i < _grid_size; ++i) {
    if (_label[i] != kEmptyGridLabel) {
      (*num_cells_per_segment)[_label[i]]++;
    }
  }
  for (int i = 0; i < _num_points; ++i) {
    const int idx = _point_cloud_grid_idx[i];
    if (idx >= 0) {
      (*segments_indices)[_label[idx]].push_back(i);
    }
  }
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
