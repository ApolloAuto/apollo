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
#include "i_ground.h"
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <iostream>

namespace idl {
void PlaneFitGroundDetectorParam::set_default() {
  nr_points_max = 320000;  // assume max 320000 points
  nr_grids_fine = 256;     // must be 2 and above
  nr_grids_coarse = 16;    // must be 2 and above
  nr_z_comp_candis = 32;
  nr_z_comp_fail_threshold = 4;
  nr_inliers_min_threshold = 8;
  nr_samples_min_threshold = 128;
  nr_samples_max_threshold = 1024;
  sample_region_z_lower = -3.0f;  //-3.0m
  sample_region_z_upper = -1.0f;  //-1.0m
  roi_region_rad_x = 72.0f;       // 72.0f
  roi_region_rad_y = 72.0f;       // 72.0f
  roi_region_rad_z = 10.0f;
  roi_near_rad = 32.0f;                   // near range: 32m;
  planefit_dist_threshold_near = 0.10f;   // 10.cm
  planefit_dist_threshold_far = 0.20f;    // 20.0cm
  planefit_filter_threshold = 0.10f;      // 10.0cm;
  planefit_orien_threshold = 5.0f;        // 5 degree
  termi_inlier_percen_threshold = 0.99f;  // 99%
  nr_ransac_iter_threshold = 32;
  candidate_filter_threshold = 1.0f;  // 1 meter
  nr_smooth_iter = 1;
}

bool PlaneFitGroundDetectorParam::validate() const {
  if (nr_grids_coarse < 2 || nr_grids_fine < 2 ||
      nr_grids_coarse > nr_grids_fine || nr_points_max == 0 ||
      nr_samples_min_threshold == 0 || nr_samples_max_threshold == 0 ||
      nr_inliers_min_threshold == 0 || nr_ransac_iter_threshold == 0 ||
      roi_region_rad_x <= 0.f || roi_region_rad_y <= 0.f ||
      roi_region_rad_z <= 0.f ||
      planefit_dist_threshold_near > planefit_dist_threshold_far) {
    std::cerr << "Invalid ground detector parameters... " << std::endl;
    return false;
  }
  return true;
}

int PlaneFitPointCandIndices::prune(unsigned int min_nr_samples,
                                    unsigned int max_nr_samples) {
  assert(min_nr_samples < max_nr_samples);
  unsigned int size = indices.size();
  unsigned int half = 0;
  if (size > max_nr_samples) {
    i_randomized_shuffle1(indices.data(), (int)indices.size(), random_seed);
    // i_random_sample(indices.data(), max_nr_samples, (int)size, random_seed);
    // for (int i = 0; i < max_nr_samples; ++i) {
    //    int j = indices[i];
    //    //i <= j forever;
    //    indices[i] = indices[j];
    //}
    indices.resize(max_nr_samples);
  } else {
    if (size > min_nr_samples) {
      half = size >> 1;
      while (half > min_nr_samples) {
        half = half >> 1;
      }
      size = half << 1;
      i_randomized_shuffle1(indices.data(), (int)indices.size(), random_seed);
      indices.resize(size);
    }
  }
  return (int)indices.size();
}

PlaneFitGroundDetector::PlaneFitGroundDetector(
    const PlaneFitGroundDetectorParam &param)
    : BaseGroundDetector(param) {
  bool is_initialized = init();
  assert(is_initialized);
}

PlaneFitGroundDetector::~PlaneFitGroundDetector() { cleanup(); }

// init the order lookup table
void PlaneFitGroundDetector::init_order_table(const VoxelGridXY<float> *vg,
                                              std::pair<int, int> *order) {
  std::vector<std::pair<float, int>> map_dist;
  float cx = 0.f;
  float cy = 0.f;
  float dist2 = 0.f;
  float radius = 0.f;
  unsigned int i = 0;
  int id = 0;
  for (i = 0; i < vg->nr_voxel(); ++i) {
    const auto &voxel = vg->get_const_voxels()[i];
    radius = voxel._dim_x * 0.5;
    cx = voxel._v[0] + radius;
    cy = voxel._v[1] + radius;
    dist2 = cx * cx + cy * cy;
    map_dist.push_back(std::pair<float, int>(dist2, i));
  }
  sort(map_dist.begin(), map_dist.end(),
       [](const std::pair<float, int> &a, const std::pair<float, int> &b) {
         return a.first < b.first;
       });
  for (i = 0; i < map_dist.size(); ++i) {
    id = map_dist[i].second;
    const auto &voxel = vg->get_const_voxels()[id];
    order[i].first = voxel._iy;
    order[i].second = voxel._ix;
  }
}

bool PlaneFitGroundDetector::init() {
  unsigned int r = 0;
  unsigned int c = 0;
  unsigned int pr = 0;
  unsigned int pc = 0;
  unsigned int index = 0;
  unsigned int capacity = 0;
  unsigned int sf = _param.nr_grids_fine / _param.nr_grids_coarse;
  if (!_param.validate()) {
    return false;
  }
  // fine grid:
  _vg_fine = new VoxelGridXY<float>();
  if (_vg_fine == NULL) {
    return false;
  }
  if (!_vg_fine->alloc(_param.nr_grids_fine, _param.nr_grids_fine,
                       -_param.roi_region_rad_x, _param.roi_region_rad_x,
                       -_param.roi_region_rad_y, _param.roi_region_rad_y,
                       -_param.roi_region_rad_z, _param.roi_region_rad_z)) {
    return false;
  }
  // coarse grid:
  _vg_coarse = new VoxelGridXY<float>();
  if (_vg_coarse == NULL) {
    return false;
  }
  if (!_vg_coarse->alloc(_param.nr_grids_coarse, _param.nr_grids_coarse,
                         -_param.roi_region_rad_x, _param.roi_region_rad_x,
                         -_param.roi_region_rad_y, _param.roi_region_rad_y,
                         -_param.roi_region_rad_z, _param.roi_region_rad_z)) {
    return false;
  }

  // init order lookup table
  _order_table = i_alloc<std::pair<int, int>>(_vg_fine->nr_voxel());
  init_order_table(_vg_coarse, _order_table);

  // ground plane:
  _ground_planes = i_alloc2<GroundPlaneLiDAR>(_param.nr_grids_coarse,
                                              _param.nr_grids_coarse);
  if (!_ground_planes) {
    return false;
  }
  _ground_planes_sphe = i_alloc2<GroundPlaneSpherical>(_param.nr_grids_coarse,
                                                       _param.nr_grids_coarse);
  if (!_ground_planes_sphe) {
    return false;
  }
  _ground_z = i_alloc2<std::pair<float, bool>>(_param.nr_grids_coarse,
                                               _param.nr_grids_coarse);
  if (!_ground_z) {
    return false;
  }
  // sample candis:
  _local_candis = i_alloc2<PlaneFitPointCandIndices>(_param.nr_grids_coarse,
                                                     _param.nr_grids_coarse);
  if (!_local_candis) {
    return false;
  }
  // reserve space to avoid runtime memory re-allocation
  for (r = 0; r < _param.nr_grids_coarse; ++r) {
    for (c = 0; c < _param.nr_grids_coarse; ++c) {
      if (r < (_param.nr_grids_coarse << 3) &&
          c < (_param.nr_grids_coarse << 3)) {
        capacity = 16384;
      } else if (r < (_param.nr_grids_coarse << 2) &&
                 c < (_param.nr_grids_coarse << 2)) {
        capacity = 8192;
      } else if (r < (_param.nr_grids_coarse << 1) &&
                 c < (_param.nr_grids_coarse << 1)) {
        capacity = 4096;
      } else {
        capacity = 512;
      }
      _local_candis[r][c].reserve(capacity);
    }
  }
  // threeds in ransac, in inhomogeneous coordinates:
  _pf_threeds =
      i_alloc_aligned<float>(_param.nr_samples_max_threshold * _dim_point, 4);
  if (!_pf_threeds) {
    return false;
  }
  memset((void *)_pf_threeds, 0,
         _param.nr_samples_max_threshold * _dim_point * sizeof(float));
  // labels:
  _labels = i_alloc_aligned<char>(_param.nr_points_max, 4);
  if (!_labels) {
    return false;
  }
  memset((void *)_labels, 0, _param.nr_points_max * sizeof(char));
  // map of fine grid id to coarse id:
  _map_fine_to_coarse = i_alloc_aligned<unsigned int>(
      _param.nr_grids_fine * _param.nr_grids_fine, 4);
  if (!_map_fine_to_coarse) {
    return false;
  }
  for (r = 0; r < _param.nr_grids_fine; ++r) {
    pr = r / sf;
    index = r * _param.nr_grids_fine;
    for (c = 0; c < _param.nr_grids_fine; ++c) {
      pc = c / sf;
      _map_fine_to_coarse[index + c] = pr * _param.nr_grids_coarse + pc;
    }
  }
  // ransac memory:
  _sampled_z_values = i_alloc_aligned<float>(_param.nr_z_comp_candis, 4);
  if (!_sampled_z_values) {
    return false;
  }
  memset((void *)_sampled_z_values, 0, _param.nr_z_comp_candis * sizeof(float));
  // ransac memory:
  _sampled_indices = i_alloc_aligned<int>(_param.nr_z_comp_candis, 4);
  if (!_sampled_indices) {
    return false;
  }
  memset((void *)_sampled_indices, 0, _param.nr_z_comp_candis * sizeof(int));
  // ransac thresholds:
  _pf_thresholds =
      i_alloc2<float>(_param.nr_grids_coarse, _param.nr_grids_coarse);
  if (!_pf_thresholds) {
    return false;
  }
  // compute thresholds
  compute_adaptive_threshold();
  return true;
}

void PlaneFitGroundDetector::cleanup() {
  if (_vg_fine) {
    delete _vg_fine;
  }
  if (_vg_coarse) {
    delete _vg_coarse;
  }
  i_free2<GroundPlaneLiDAR>(_ground_planes);
  i_free2<GroundPlaneSpherical>(_ground_planes_sphe);
  i_free2<std::pair<float, bool>>(_ground_z);
  i_free2<PlaneFitPointCandIndices>(_local_candis);
  i_free_aligned<float>(_pf_threeds);
  i_free_aligned<char>(_labels);
  i_free_aligned<unsigned int>(_map_fine_to_coarse);
  i_free_aligned<float>(_sampled_z_values);
  i_free_aligned<int>(_sampled_indices);
  i_free2<float>(_pf_thresholds);
  i_free<std::pair<int, int>>(_order_table);
}

int PlaneFitGroundDetector::compare_z(const float *point_cloud,
                                      const std::vector<int> &indices,
                                      const float *z_values,
                                      PlaneFitPointCandIndices &candi,
                                      unsigned int nr_points,
                                      unsigned int nr_point_element,
                                      unsigned int nr_compares) {
  int pos = 0;
  int nr_candis = 0;
  unsigned int i = 0;
  unsigned int nr_contradi = 0;
  unsigned int nr_z_comp_fail_threshold =
      i_min(_param.nr_z_comp_fail_threshold, (unsigned int)(nr_compares >> 1));
  const float *ptr = NULL;
  float z = 0;
  float delta_z = 0;
  std::vector<int>::const_iterator iter = indices.cbegin();
  while (iter < indices.cend()) {
    nr_contradi = 0;
    pos = *iter++;
    assert(pos < (int)nr_points);
    nr_contradi = 0;
    // requires the Z element to be in the third position, i.e., after X, Y
    ptr = point_cloud + (pos * nr_point_element);
    z = ptr[2];
    // for near range check valid height
    if (i_abs(ptr[0]) < _param.roi_near_rad &&
        i_abs(ptr[1]) < _param.roi_near_rad) {
      if (z < _param.sample_region_z_lower ||
          z > _param.sample_region_z_upper) {
        continue;
      }
    }
    if (nr_compares > nr_z_comp_fail_threshold) {
      for (i = 0; i < nr_compares; ++i) {
        delta_z = i_abs(z_values[i] - z);
        if (delta_z > _param.planefit_filter_threshold) {
          nr_contradi++;
          if (nr_contradi > nr_z_comp_fail_threshold) {
            break;
          }
        }
      }
    }
    if (nr_contradi <= nr_z_comp_fail_threshold) {
      _labels[pos] = 1;
      candi.push_index(pos);
      nr_candis++;
    }
  }
  return nr_candis;
}

void PlaneFitGroundDetector::compute_adaptive_threshold() {
  unsigned int r = 0;
  unsigned int c = 0;
  float dr = 0;
  float dc = 0;
  float k = 0;
  float b = 0;
  float min_dist = 0;
  float max_dist = 0;
  float thre = 0;
  float grid_rad = (float)(_param.nr_grids_coarse - 1) / 2;
  assert(_pf_thresholds != NULL);
  for (r = 0; r < _param.nr_grids_coarse; ++r) {
    dr = (float)r - grid_rad;
    dr *= dr;
    for (c = 0; c < _param.nr_grids_coarse; ++c) {
      dc = (float)c - grid_rad;
      dc *= dc;
      // store to center distance:
      _pf_thresholds[r][c] = i_sqrt(dr + dc);
    }
  }
  i_min_max_elements(_pf_thresholds[0],
                     _param.nr_grids_coarse * _param.nr_grids_coarse, min_dist,
                     max_dist);
  if (max_dist - min_dist < Constant<float>::EPSILON()) {
    thre = (_param.planefit_dist_threshold_near +
            _param.planefit_dist_threshold_far) /
           2;
    for (r = 0; r < _param.nr_grids_coarse; ++r) {
      for (c = 0; c < _param.nr_grids_coarse; ++c) {
        _pf_thresholds[r][c] = thre;
      }
    }
  } else {
    k = _param.planefit_dist_threshold_far -
        _param.planefit_dist_threshold_near;
    k = k / (max_dist - min_dist);
    b = _param.planefit_dist_threshold_far +
        _param.planefit_dist_threshold_near;
    b = (b - k * (min_dist + max_dist)) / 2;
    for (r = 0; r < _param.nr_grids_coarse; ++r) {
      for (c = 0; c < _param.nr_grids_coarse; ++c) {
        thre = k * _pf_thresholds[r][c] + b;
        _pf_thresholds[r][c] = thre;
      }
    }
  }
}

void PlaneFitGroundDetector::compute_signed_ground_height(
    const float *point_cloud, float *height_above_ground,
    unsigned int nr_points, unsigned int nr_point_elements) {
  unsigned int r = 0;
  unsigned int nm1 = _param.nr_grids_coarse - 1;
  for (r = 0; r < nr_points; ++r) {
    height_above_ground[r] = FLT_MAX;
  }
  compute_signed_ground_height_line(
      point_cloud, _ground_planes[0], _ground_planes[0], _ground_planes[1],
      height_above_ground, 0, nr_points, nr_point_elements);
  for (r = 1; r < nm1; ++r) {
    compute_signed_ground_height_line(point_cloud, _ground_planes[r - 1],
                                      _ground_planes[r], _ground_planes[r + 1],
                                      height_above_ground, r, nr_points,
                                      nr_point_elements);
  }
  compute_signed_ground_height_line(point_cloud, _ground_planes[nm1 - 1],
                                    _ground_planes[nm1], _ground_planes[nm1],
                                    height_above_ground, nm1, nr_points,
                                    nr_point_elements);
}

void PlaneFitGroundDetector::compute_signed_ground_height_line(
    const float *point_cloud, const GroundPlaneLiDAR *up,
    const GroundPlaneLiDAR *cn, const GroundPlaneLiDAR *dn,
    float *height_above_ground, unsigned int r, unsigned int nr_points,
    unsigned int nr_point_elements) {
  unsigned int i = 0;
  unsigned int c = 0;
  unsigned int id = 0;
  int pos = 0;
  char label = 0;
  const float *cptr = NULL;
  float dist[] = {0, 0, 0, 0, 0};
  const float *plane[] = {NULL, NULL, NULL, NULL, NULL};
  float min_abs_dist = 0;
  unsigned int nm1 = _param.nr_grids_coarse - 1;
  assert(_param.nr_grids_coarse >= 2);
  plane[0] = cn[0].is_valid() ? cn[0].params : NULL;
  plane[1] = cn[1].is_valid() ? cn[1].params : NULL;
  plane[2] = up[0].is_valid() ? up[0].params : NULL;
  plane[3] = dn[0].is_valid() ? dn[0].params : NULL;
  std::vector<int>::const_iterator iter = (*_vg_coarse)(r, 0)._indices.cbegin();
  while (iter < (*_vg_coarse)(r, 0)._indices.cend()) {
    pos = *iter;
    assert(pos < (int)nr_points);
    label = _labels[pos];
    cptr = point_cloud + (nr_point_elements * pos);
    dist[0] = plane[0] != NULL
                  ? i_plane_to_point_signed_distance_w_unit_norm(plane[0], cptr)
                  : FLT_MAX;
    min_abs_dist = i_abs(dist[0]);
    id = 0;
    // for candidates we take min dist:
    if (label) {
      dist[1] = plane[1] != NULL ? i_plane_to_point_signed_distance_w_unit_norm(
                                       plane[1], cptr)
                                 : FLT_MAX;
      dist[2] = plane[2] != NULL ? i_plane_to_point_signed_distance_w_unit_norm(
                                       plane[2], cptr)
                                 : FLT_MAX;
      dist[3] = plane[3] != NULL ? i_plane_to_point_signed_distance_w_unit_norm(
                                       plane[3], cptr)
                                 : FLT_MAX;
      for (i = 1; i < 4; ++i) {
        if (min_abs_dist > i_abs(dist[i])) {
          min_abs_dist = i_abs(dist[i]);
          id = i;
        }
      }
    }
    height_above_ground[pos] = dist[id];
    ++iter;
  }

  for (c = 1; c < nm1; ++c) {
    plane[0] = cn[c].is_valid() ? cn[c].params : NULL;
    plane[1] = cn[c - 1].is_valid() ? cn[c - 1].params : NULL;
    plane[2] = cn[c + 1].is_valid() ? cn[c + 1].params : NULL;
    plane[3] = up[c].is_valid() ? up[c].params : NULL;
    plane[4] = dn[c].is_valid() ? dn[c].params : NULL;
    iter = (*_vg_coarse)(r, c)._indices.cbegin();
    while (iter < (*_vg_coarse)(r, c)._indices.cend()) {
      pos = *iter;
      assert(pos < (int)nr_points);
      label = _labels[pos];
      cptr = point_cloud + (nr_point_elements * pos);
      dist[0] = plane[0] != NULL ? i_plane_to_point_signed_distance_w_unit_norm(
                                       plane[0], cptr)
                                 : FLT_MAX;
      min_abs_dist = i_abs(dist[0]);
      id = 0;
      if (label) {
        dist[1] =
            plane[1] != NULL
                ? i_plane_to_point_signed_distance_w_unit_norm(plane[1], cptr)
                : FLT_MAX;
        dist[2] =
            plane[2] != NULL
                ? i_plane_to_point_signed_distance_w_unit_norm(plane[2], cptr)
                : FLT_MAX;
        dist[3] =
            plane[3] != NULL
                ? i_plane_to_point_signed_distance_w_unit_norm(plane[3], cptr)
                : FLT_MAX;
        dist[4] =
            plane[4] != NULL
                ? i_plane_to_point_signed_distance_w_unit_norm(plane[4], cptr)
                : FLT_MAX;
        for (i = 1; i < 5; ++i) {
          if (min_abs_dist > i_abs(dist[i])) {
            min_abs_dist = i_abs(dist[i]);
            id = i;
          }
        }
      }
      height_above_ground[pos] = dist[id];
      ++iter;
    }
  }
  plane[0] = cn[nm1].is_valid() ? cn[nm1].params : NULL;
  plane[1] = cn[nm1 - 1].is_valid() ? cn[nm1 - 1].params : NULL;
  plane[2] = up[nm1].is_valid() ? up[nm1].params : NULL;
  plane[3] = dn[nm1].is_valid() ? dn[nm1].params : NULL;
  iter = (*_vg_coarse)(r, nm1)._indices.cbegin();
  while (iter < (*_vg_coarse)(r, nm1)._indices.cend()) {
    pos = *iter;
    assert(pos < (int)nr_points);
    label = _labels[pos];
    cptr = point_cloud + (nr_point_elements * pos);
    dist[0] = plane[0] != NULL
                  ? i_plane_to_point_signed_distance_w_unit_norm(plane[0], cptr)
                  : FLT_MAX;
    min_abs_dist = i_abs(dist[0]);
    id = 0;
    // for candidates we take min dist:
    if (label) {
      dist[1] = plane[1] != NULL ? i_plane_to_point_signed_distance_w_unit_norm(
                                       plane[1], cptr)
                                 : FLT_MAX;
      dist[2] = plane[2] != NULL ? i_plane_to_point_signed_distance_w_unit_norm(
                                       plane[2], cptr)
                                 : FLT_MAX;
      dist[3] = plane[3] != NULL ? i_plane_to_point_signed_distance_w_unit_norm(
                                       plane[3], cptr)
                                 : FLT_MAX;
      for (i = 1; i < 4; ++i) {
        if (min_abs_dist > i_abs(dist[i])) {
          min_abs_dist = i_abs(dist[i]);
          id = i;
        }
      }
    }
    height_above_ground[pos] = dist[id];
    ++iter;
  }
}

int PlaneFitGroundDetector::filter_grid(const Voxel<float> &vx,
                                        const float *point_cloud,
                                        PlaneFitPointCandIndices &candi,
                                        unsigned int nr_points,
                                        unsigned int nr_point_element) {
  int pos = 0;
  int rseed = I_DEFAULT_SEED;
  int nr_candis = 0;
  unsigned int i = 0;
  unsigned int nr_samples = i_min(_param.nr_z_comp_candis, vx.nr_points());
  if (vx.empty()) {
    return 0;
  }
  // generate sampled indices
  if (vx.nr_points() <= _param.nr_z_comp_candis) {
    // i_ramp(_sampled_indices, vx.nr_points());
    // sampled z values
    for (i = 0; i < vx.nr_points(); ++i) {
      pos = vx._indices[i] * nr_point_element;
      // requires the Z element to be in the third position, i.e., after X, Y
      _sampled_z_values[i] = (point_cloud + pos)[2];
    }
  } else {
    i_random_sample(_sampled_indices, (int)_param.nr_z_comp_candis,
                    (int)vx.nr_points(), rseed);
    // sampled z values
    for (i = 0; i < nr_samples; ++i) {
      pos = vx._indices[_sampled_indices[i]] * nr_point_element;
      // requires the Z element to be in the third position, i.e., after X, Y
      _sampled_z_values[i] = (point_cloud + pos)[2];
    }
  }
  // filter points and get plane fitting candidates
  nr_candis = compare_z(point_cloud, vx._indices, _sampled_z_values, candi,
                        nr_points, nr_point_element, nr_samples);
  return nr_candis;
}

int PlaneFitGroundDetector::filter_line(unsigned int r) {
  int nr_candis = 0;
  unsigned int c = 0;
  const float *point_cloud = _vg_fine->const_data();
  unsigned int nr_points = _vg_fine->nr_points();
  unsigned int nr_point_element = _vg_fine->nr_point_element();
  unsigned int begin = (r * _param.nr_grids_fine);
  int parent = 0;
  for (c = 0; c < _param.nr_grids_fine; c++) {
    parent = _map_fine_to_coarse[begin + c];
    nr_candis +=
        filter_grid((*_vg_fine)(r, c), point_cloud, _local_candis[0][parent],
                    nr_points, nr_point_element);
  }
  return nr_candis;
}

int PlaneFitGroundDetector::filter() {
  int nr_candis = 0;
  unsigned int i = 0;
  unsigned int r = 0;
  memset((void *)_labels, 0, _vg_fine->nr_points() * sizeof(char));
  // clear candidate list
  for (i = 0; i < _vg_coarse->nr_voxel(); ++i) {
    _local_candis[0][i].clear();
  }
  // filter plane fitting candidates
  for (r = 0; r < _param.nr_grids_fine; ++r) {
    nr_candis += filter_line(r);
  }
  return nr_candis;
}

int PlaneFitGroundDetector::fit_grid(const float *point_cloud,
                                     PlaneFitPointCandIndices &candi,
                                     GroundPlaneLiDAR &groundplane,
                                     unsigned int nr_points,
                                     unsigned int nr_point_element,
                                     float dist_thre) {
  // initialize the best plane
  groundplane.force_invalid();
  // not enough samples, failed and return
  if (candi.size() < _param.nr_inliers_min_threshold) {
    return (0);
  }
  GroundPlaneLiDAR plane;
  float ptp_dist = 0;
  float fit_cost = 0;
  float fit_cost_best = dist_thre;
  int nr_inliers = 0;
  int nr_inliers_best = -1;
  int i = 0;
  int j = 0;
  int rseed = I_DEFAULT_SEED;
  int indices_trial[] = {0, 0, 0};
  int nr_samples = candi.prune(_param.nr_samples_min_threshold,
                               _param.nr_samples_max_threshold);
  int nr_inliers_termi =
      i_round(nr_samples * _param.termi_inlier_percen_threshold);
  // 3x3 matrix stores: x, y, z; x, y, z; x, y, z;
  float samples[9];
  // copy 3D points
  float *psrc = NULL;
  float *pdst = _pf_threeds;
  for (i = 0; i < nr_samples; ++i) {
    assert(candi[i] < (int)nr_points);
    i_copy3(point_cloud + (nr_point_element * candi[i]), pdst);
    pdst += _dim_point;
  }
  // generate plane hypothesis and vote
  for (i = 0; i < _param.nr_ransac_iter_threshold; ++i) {
    i_random_sample(indices_trial, 3, nr_samples, rseed);
    i_scale3(indices_trial, _dim_point);
    i_copy3(_pf_threeds + indices_trial[0], samples);
    i_copy3(_pf_threeds + indices_trial[1], samples + 3);
    i_copy3(_pf_threeds + indices_trial[2], samples + 6);
    i_plane_fit_destroyed(samples, plane.params);
    // check if the plane hypothesis has valid geometry
    if (plane.get_degree_normal_to_z() > _param.planefit_orien_threshold) {
      continue;
    }
    // iterate samples and check if the point to plane distance is below
    // threshold
    psrc = _pf_threeds;
    nr_inliers = 0;
    fit_cost = 0;
    for (j = 0; j < nr_samples; ++j) {
      ptp_dist = i_plane_to_point_distance_w_unit_norm(plane.params, psrc);
      if (ptp_dist < dist_thre) {
        nr_inliers++;
        fit_cost += ptp_dist;
      }
      psrc += _dim_point;
    }
    // assign number of supports
    plane.set_nr_support(nr_inliers);

    fit_cost = nr_inliers > 0 ? (fit_cost / nr_inliers) : dist_thre;
    // record the best plane
    if (nr_inliers >= nr_inliers_best) {
      if (nr_inliers == nr_inliers_best) {
        if (fit_cost < fit_cost_best) {
          nr_inliers_best = nr_inliers;
          fit_cost_best = fit_cost;
          groundplane = plane;
        }
      } else {
        nr_inliers_best = nr_inliers;
        fit_cost_best = fit_cost;
        groundplane = plane;
      }
      // found enough inliers - terminate the ransac
      if (nr_inliers_best > nr_inliers_termi) {
        break;
      }
    }
  }
  // check if meet the inlier number requirement
  if (!groundplane.is_valid()) {
    return (0);
  }
  if (groundplane.get_nr_support() < (int)_param.nr_inliers_min_threshold) {
    groundplane.force_invalid();
    return (0);
  }
  // iterate samples and check if the point to plane distance is within
  // threshold
  nr_inliers = 0;
  psrc = _pf_threeds;
  pdst = _pf_threeds;
  for (i = 0; i < nr_samples; ++i) {
    ptp_dist = i_plane_to_point_distance_w_unit_norm(groundplane.params, psrc);
    if (ptp_dist < dist_thre) {
      i_copy3(psrc, pdst);
      pdst += 3;
      nr_inliers++;
    }
    psrc += _dim_point;
  }
  groundplane.set_nr_support(nr_inliers);
  // note that _pf_threeds will be destroyed after calling this routine
  i_plane_fit_total_least_square(_pf_threeds, groundplane.params, nr_inliers);
  // filtering: the best plane orientation is not valid*/
  // std::cout << groundplane.get_degree_normal_to_z() << std::endl;
  if (groundplane.get_degree_normal_to_z() > _param.planefit_orien_threshold) {
    groundplane.force_invalid();
    return (0);
  }
  return nr_inliers;
}

int PlaneFitGroundDetector::fit_line(unsigned int r) {
  int nr_grids = 0;
  unsigned int c = 0;
  GroundPlaneLiDAR gp;
  for (c = 0; c < _param.nr_grids_coarse; c++) {
    if (fit_grid(_vg_coarse->const_data(), _local_candis[r][c], gp,
                 _vg_coarse->nr_points(), _vg_coarse->nr_point_element(),
                 _pf_thresholds[r][c]) >=
        (int)_param.nr_inliers_min_threshold) {
      // transform to polar coordinates and store:
      i_plane_eucli_to_spher(gp, _ground_planes_sphe[r][c]);
      _ground_planes[r][c] = gp;
      nr_grids++;
    } else {
      _ground_planes_sphe[r][c].force_invalid();
      _ground_planes[r][c].force_invalid();
    }
  }
  return nr_grids;
}

int PlaneFitGroundDetector::fit() {
  int nr_grids = 0;
  for (unsigned int r = 0; r < _param.nr_grids_coarse; ++r) {
    nr_grids += fit_line(r);
  }
  return nr_grids;
}

// filter candidates by neighbors
int PlaneFitGroundDetector::filter_candidates(
    int r, int c, const float *point_cloud, PlaneFitPointCandIndices &candi,
    std::vector<std::pair<int, int>> &neighbors,
    unsigned int nr_point_element) {
  float avg_z = 0.f;
  int count = 0;
  unsigned int i = 0;
  int r_n = 0;
  int c_n = 0;
  float z = 0.f;
  std::vector<int> filtered_indices;
  filtered_indices.reserve(candi.size());
  for (i = 0; i < neighbors.size(); ++i) {
    r_n = neighbors[i].first;
    c_n = neighbors[i].second;
    if (_ground_z[r_n][c_n].second) {
      avg_z += _ground_z[r_n][c_n].first;
      count++;
    }
  }
  if (count > 0) {
    avg_z /= count;
    _ground_z[r][c].first = avg_z;
    _ground_z[r][c].second = true;
    for (i = 0; i < candi.size(); ++i) {
      z = (point_cloud + (nr_point_element * candi[i]))[2];
      if (z > avg_z - _param.candidate_filter_threshold &&
          z < avg_z + _param.candidate_filter_threshold) {
        filtered_indices.push_back(candi[i]);
      } else {
        _labels[candi[i]] = 0;
      }
    }
    if (filtered_indices.size() != candi.size()) {
      candi.indices.assign(filtered_indices.begin(), filtered_indices.end());
    }
  }
  return count;
}

inline float calculate_two_angles(const GroundPlaneLiDAR &p1,
                                  const GroundPlaneLiDAR &p2) {
  float numerator = i_dot3(p1.params, p2.params);
  float denominator = i_l2_norm(p1.params, 3) * i_l2_norm(p2.params, 3);
  return i_acos(numerator * i_rec(denominator));
}

int PlaneFitGroundDetector::fit_grid_with_neighbors(
    int r, int c, const float *point_cloud, GroundPlaneLiDAR &groundplane,
    unsigned int nr_points, unsigned int nr_point_element, float dist_thre) {
  // initialize the best plane
  groundplane.force_invalid();
  // not enough samples, failed and return

  PlaneFitPointCandIndices &candi = _local_candis[r][c];
  std::vector<std::pair<int, int>> neighbors;
  get_neighbors(r, c, _param.nr_grids_coarse, _param.nr_grids_coarse,
                neighbors);
  filter_candidates(r, c, point_cloud, candi, neighbors, nr_point_element);

  if (candi.size() < _param.nr_inliers_min_threshold) {
    return (0);
  }

  GroundPlaneLiDAR plane;
  int nr_iter = _param.nr_ransac_iter_threshold + neighbors.size();
  GroundPlaneLiDAR hypothesis[nr_iter];
  float ptp_dist = 0;
  int best = -1;
  int nr_inliers = 0;
  int nr_inliers_best = -1;
  float angle_best = FLT_MAX;
  int i = 0;
  int j = 0;
  int rseed = I_DEFAULT_SEED;
  int indices_trial[] = {0, 0, 0};
  int nr_samples = candi.prune(_param.nr_samples_min_threshold,
                               _param.nr_samples_max_threshold);
  int nr_inliers_termi =
      i_round(nr_samples * _param.termi_inlier_percen_threshold);
  // 3x3 matrix stores: x, y, z; x, y, z; x, y, z;
  float samples[9];
  // copy 3D points
  float *psrc = NULL;
  float *pdst = _pf_threeds;
  int r_n = 0;
  int c_n = 0;
  float angle = -1.f;
  for (i = 0; i < nr_samples; ++i) {
    assert(candi[i] < (int)nr_points);
    i_copy3(point_cloud + (nr_point_element * candi[i]), pdst);
    pdst += _dim_point;
  }
  // generate plane hypothesis and vote
  for (i = 0; i < _param.nr_ransac_iter_threshold; ++i) {
    i_random_sample(indices_trial, 3, nr_samples, rseed);
    i_scale3(indices_trial, _dim_point);
    i_copy3(_pf_threeds + indices_trial[0], samples);
    i_copy3(_pf_threeds + indices_trial[1], samples + 3);
    i_copy3(_pf_threeds + indices_trial[2], samples + 6);
    i_plane_fit_destroyed(samples, hypothesis[i].params);
    // check if the plane hypothesis has valid geometry
    if (hypothesis[i].get_degree_normal_to_z() >
        _param.planefit_orien_threshold) {
      continue;
    }
    // iterate samples and check if the point to plane distance is below
    // threshold
    psrc = _pf_threeds;
    nr_inliers = 0;
    for (j = 0; j < nr_samples; ++j) {
      ptp_dist =
          i_plane_to_point_distance_w_unit_norm(hypothesis[i].params, psrc);
      if (ptp_dist < dist_thre) {
        nr_inliers++;
      }
      psrc += _dim_point;
    }
    // assign number of supports
    hypothesis[i].set_nr_support(nr_inliers);

    if (nr_inliers > nr_inliers_termi) {
      break;
    }
  }

  for (i = 0; i < neighbors.size(); ++i) {
    r_n = neighbors[i].first;
    c_n = neighbors[i].second;
    if (_ground_planes[r_n][c_n].is_valid()) {
      hypothesis[i + _param.nr_ransac_iter_threshold] =
          _ground_planes[r_n][c_n];
      psrc = _pf_threeds;
      nr_inliers = 0;
      for (j = 0; j < nr_samples; ++j) {
        ptp_dist = i_plane_to_point_distance_w_unit_norm(
            hypothesis[i + _param.nr_ransac_iter_threshold].params, psrc);
        if (ptp_dist < dist_thre) {
          nr_inliers++;
        }
        psrc += _dim_point;
      }
      if (nr_inliers < (int)_param.nr_inliers_min_threshold) {
        hypothesis[i + _param.nr_ransac_iter_threshold].force_invalid();
        continue;
      }
      hypothesis[i + _param.nr_ransac_iter_threshold].set_nr_support(
          nr_inliers);
    }
  }

  nr_inliers_best = -1;
  for (i = 0; i < nr_iter; ++i) {
    if (!(hypothesis[i].is_valid())) {
      continue;
    }
    nr_inliers = hypothesis[i].get_nr_support();
    if (nr_inliers >= nr_inliers_best) {
      if (nr_inliers == nr_inliers_best) {
        angle = calculate_angle_dist(hypothesis[i], neighbors);
        if (angle < angle_best && angle > 0) {
          angle_best = angle;
          best = i;
        }
      } else {
        nr_inliers_best = nr_inliers;
        angle_best = calculate_angle_dist(hypothesis[i], neighbors);
        best = i;
      }
    }
  }

  groundplane = hypothesis[best];

  // check if meet the inlier number requirement
  if (!groundplane.is_valid()) {
    return (0);
  }
  if (groundplane.get_nr_support() < (int)_param.nr_inliers_min_threshold) {
    groundplane.force_invalid();
    return (0);
  }
  // iterate samples and check if the point to plane distance is within
  // threshold
  nr_inliers = 0;
  psrc = _pf_threeds;
  pdst = _pf_threeds;
  for (i = 0; i < nr_samples; ++i) {
    ptp_dist = i_plane_to_point_distance_w_unit_norm(groundplane.params, psrc);
    if (ptp_dist < dist_thre) {
      i_copy3(psrc, pdst);
      pdst += 3;
      nr_inliers++;
    }
    psrc += _dim_point;
  }
  groundplane.set_nr_support(nr_inliers);

  // note that _pf_threeds will be destroyed after calling this routine
  i_plane_fit_total_least_square(_pf_threeds, groundplane.params, nr_inliers);
  if (angle_best <= calculate_angle_dist(groundplane, neighbors)) {
    groundplane = hypothesis[best];
    groundplane.set_status(true);
  }

  if (groundplane.get_degree_normal_to_z() > _param.planefit_orien_threshold) {
    groundplane.force_invalid();
    return (0);
  }

  const auto &voxel_cur = (*_vg_coarse)(r, c);
  float radius = voxel_cur._dim_x * 0.5;
  float cx = voxel_cur._v[0] + radius;
  float cy = voxel_cur._v[1] + radius;
  float cz = -(groundplane.params[0] * cx + groundplane.params[1] * cy +
               groundplane.params[3]) /
             groundplane.params[2];
  _ground_z[r][c].first = cz;
  _ground_z[r][c].second = true;

  return nr_inliers;
}

float PlaneFitGroundDetector::calculate_angle_dist(
    const GroundPlaneLiDAR &plane,
    const std::vector<std::pair<int, int>> &neighbors) {
  float angle_dist = 0;
  int count = 0;
  unsigned int j = 0;
  int r_n = 0;
  int c_n = 0;
  for (j = 0; j < neighbors.size(); ++j) {
    r_n = neighbors[j].first;
    c_n = neighbors[j].second;
    if (_ground_planes[r_n][c_n].is_valid()) {
      angle_dist += calculate_two_angles(_ground_planes[r_n][c_n], plane);
      count++;
    }
  }
  if (count == 0) {
    return -1;
  }
  return angle_dist / count;
}

int PlaneFitGroundDetector::fit_in_order() {
  int nr_grids = 0;
  unsigned int i = 0;
  unsigned int j = 0;
  int r = 0;
  int c = 0;
  GroundPlaneLiDAR gp;
  for (i = 0; i < _param.nr_grids_coarse; ++i) {
    for (j = 0; j < _param.nr_grids_coarse; ++j) {
      _ground_z[i][j].first = 0.f;
      _ground_z[i][j].second = false;
    }
  }
  for (i = 0; i < _vg_coarse->nr_voxel(); ++i) {
    r = _order_table[i].first;
    c = _order_table[i].second;
    if (fit_grid_with_neighbors(
            r, c, _vg_coarse->const_data(), gp, _vg_coarse->nr_points(),
            _vg_coarse->nr_point_element(),
            _pf_thresholds[r][c]) >= (int)_param.nr_inliers_min_threshold) {
      i_plane_eucli_to_spher(gp, _ground_planes_sphe[r][c]);
      _ground_planes[r][c] = gp;
      nr_grids++;
    } else {
      _ground_planes_sphe[r][c].force_invalid();
      _ground_planes[r][c].force_invalid();
    }
  }
  return nr_grids;
}

void PlaneFitGroundDetector::get_neighbors(
    int r, int c, int rows, int cols,
    std::vector<std::pair<int, int>> &neighbors) {
  int left = i_max(0, c - 1);
  int right = i_min(cols - 1, c + 1);
  int up = i_max(0, r - 1);
  int down = i_min(rows - 1, r + 1);
  int x = 0;
  int y = 0;
  neighbors.reserve(8);
  for (x = left; x <= right; ++x) {
    for (y = up; y <= down; ++y) {
      if (x == c && y == r) {
        continue;
      }
      neighbors.push_back(std::pair<int, int>(y, x));
    }
  }
}

int PlaneFitGroundDetector::smooth_line(unsigned int up, unsigned int r,
                                        unsigned int dn) {
  int nr_grids = 0;
  unsigned int c = 0;
  unsigned int nm1 = _param.nr_grids_coarse - 1;
  GroundPlaneSpherical plane;
  assert(_param.nr_grids_coarse >= 2);
  assert(up < _param.nr_grids_coarse);
  assert(r < _param.nr_grids_coarse);
  assert(dn < _param.nr_grids_coarse);
  if (/*!(*_vg_coarse)(r, 0).empty()*/ true) {
    if (_ground_planes_sphe[r][0].is_valid() == false) {
      nr_grids += complete_grid(
          _ground_planes_sphe[r][0], _ground_planes_sphe[r][1],
          _ground_planes_sphe[up][0], _ground_planes_sphe[dn][0], plane);
    } else {
      nr_grids +=
          smooth_grid(_ground_planes_sphe[r][0], _ground_planes_sphe[r][0],
                      _ground_planes_sphe[r][1], _ground_planes_sphe[up][0],
                      _ground_planes_sphe[dn][0], plane);
    }
    i_plane_spher_to_eucli(plane, _ground_planes[r][0]);
  }
  for (c = 1; c < nm1; ++c) {
    if (/*!(*_vg_coarse)(r, c).empty()*/ true) {
      if (_ground_planes_sphe[r][c].is_valid() == false) {
        nr_grids += complete_grid(
            _ground_planes_sphe[r][c - 1], _ground_planes_sphe[r][c + 1],
            _ground_planes_sphe[up][c], _ground_planes_sphe[dn][c], plane);
      } else {
        nr_grids += smooth_grid(
            _ground_planes_sphe[r][c], _ground_planes_sphe[r][c - 1],
            _ground_planes_sphe[r][c + 1], _ground_planes_sphe[up][c],
            _ground_planes_sphe[dn][c], plane);
      }
      i_plane_spher_to_eucli(plane, _ground_planes[r][c]);
    }
  }
  if (/*!(*_vg_coarse)(r, nm1).empty()*/ true) {
    if (_ground_planes_sphe[r][nm1].is_valid() == false) {
      nr_grids += complete_grid(
          _ground_planes_sphe[r][nm1 - 1], _ground_planes_sphe[r][nm1],
          _ground_planes_sphe[up][nm1], _ground_planes_sphe[dn][nm1], plane);
    } else {
      nr_grids += smooth_grid(
          _ground_planes_sphe[r][nm1], _ground_planes_sphe[r][nm1 - 1],
          _ground_planes_sphe[r][nm1], _ground_planes_sphe[up][nm1],
          _ground_planes_sphe[dn][nm1], plane);
    }
    i_plane_spher_to_eucli(plane, _ground_planes[r][nm1]);
  }

  return nr_grids;
}

int PlaneFitGroundDetector::complete_grid(const GroundPlaneSpherical &lt,
                                          const GroundPlaneSpherical &rt,
                                          const GroundPlaneSpherical &up,
                                          const GroundPlaneSpherical &dn,
                                          GroundPlaneSpherical &gp) {
  int supports[] = {0, 0, 0, 0};
  float weights[] = {0.f, 0.f, 0.f, 0.f};
  gp.force_invalid();
  supports[0] = lt.get_nr_support();
  supports[1] = rt.get_nr_support();
  supports[2] = up.get_nr_support();
  supports[3] = dn.get_nr_support();
  int support_sum = i_sum4(supports);
  if (!support_sum) {
    return 0;
  }
  weights[0] = (float)(supports[0]) / support_sum;
  weights[1] = (float)(supports[1]) / support_sum;
  weights[2] = (float)(supports[2]) / support_sum;
  weights[3] = (float)(supports[3]) / support_sum;
  // weighted average:
  gp.theta = weights[0] * lt.theta + weights[1] * rt.theta +
             weights[2] * up.theta + weights[3] * dn.theta;
  gp.phi = weights[0] * lt.phi + weights[1] * rt.phi + weights[2] * up.phi +
           weights[3] * dn.phi;
  gp.d = weights[0] * lt.d + weights[1] * rt.d + weights[2] * up.d +
         weights[3] * dn.d;
  // compute average - diveided by 4, round to nearest int
  support_sum = i_max(((support_sum + 2) >> 2), 1);
  gp.set_nr_support(support_sum);
  return 1;
}

int PlaneFitGroundDetector::smooth_grid(const GroundPlaneSpherical &g,
                                        const GroundPlaneSpherical &lt,
                                        const GroundPlaneSpherical &rt,
                                        const GroundPlaneSpherical &up,
                                        const GroundPlaneSpherical &dn,
                                        GroundPlaneSpherical &gp) {
  int supports[] = {0, 0, 0, 0, 0};
  float weights[] = {0.f, 0.f, 0.f, 0.f, 0.f};
  gp.force_invalid();
  if (!g.is_valid()) {
    return 0;
  }
  // geometry weight:
  //  1
  // 1 4 1
  //  1
  if (lt.get_status()) {
    supports[0] = lt.get_nr_support();
  }
  if (rt.get_status()) {
    supports[1] = rt.get_nr_support();
  }
  if (up.get_status()) {
    supports[2] = up.get_nr_support();
  }
  if (dn.get_status()) {
    supports[3] = dn.get_nr_support();
  }
  supports[4] = (g.get_nr_support() << 2);
  int support_sum = i_sum4(supports);
  if (support_sum == 0) {
    gp = g;
    return 0;
  }
  support_sum += supports[4];
  weights[0] = (float)(supports[0]) / support_sum;
  weights[1] = (float)(supports[1]) / support_sum;
  weights[2] = (float)(supports[2]) / support_sum;
  weights[3] = (float)(supports[3]) / support_sum;
  weights[4] = (float)(supports[4]) / support_sum;
  // weighted average:
  gp.theta = weights[0] * lt.theta + weights[1] * rt.theta +
             weights[2] * up.theta + weights[3] * dn.theta +
             weights[4] * g.theta;
  gp.phi = weights[0] * lt.phi + weights[1] * rt.phi + weights[2] * up.phi +
           weights[3] * dn.phi + weights[4] * g.phi;
  gp.d = weights[0] * lt.d + weights[1] * rt.d + weights[2] * up.d +
         weights[3] * dn.d + weights[4] * g.d;
  // compute weighted average - diveided by 8, round to nearest int
  support_sum = i_max(((support_sum + 2) >> 3), 1);
  gp.set_nr_support(support_sum);
  return 1;
}

int PlaneFitGroundDetector::smooth() {
  int nr_grids = 0;
  unsigned int r = 0;
  unsigned int c = 0;
  unsigned int nm1 = _param.nr_grids_coarse - 1;
  assert(_param.nr_grids_coarse >= 2);
  nr_grids += smooth_line(0, 0, 1);
  for (r = 1; r < nm1; ++r) {
    nr_grids += smooth_line(r - 1, r, r + 1);
  }
  nr_grids += smooth_line(nm1 - 1, nm1, nm1);
  for (r = 0; r < _param.nr_grids_coarse; ++r) {
    for (c = 0; c < _param.nr_grids_coarse; ++c) {
      i_plane_eucli_to_spher(_ground_planes[r][c], _ground_planes_sphe[r][c]);
    }
  }
  return nr_grids;
}

bool PlaneFitGroundDetector::detect(const float *point_cloud,
                                    float *height_above_ground,
                                    unsigned int nr_points,
                                    unsigned int nr_point_elements) {
  assert(point_cloud != NULL);
  assert(height_above_ground != NULL);
  assert(nr_points <= _param.nr_points_max);
  assert(nr_point_elements >= 3);
  // setup the fine voxel grid
  if (!_vg_fine->set_s(point_cloud, nr_points, nr_point_elements)) {
    return false;
  }
  // setup the coarse voxel grid
  if (!_vg_coarse->set_s(point_cloud, nr_points, nr_point_elements)) {
    return false;
  }
  int nr_candis = 0;
  int nr_valid_grid = 0;
  unsigned int r = 0;
  unsigned int c = 0;
  unsigned int iter = 0;
  // filter to generate plane fitting candidates
  nr_candis = filter();
  // std::cout << "# of plane candidates: " << nr_candis << std::endl;
  // fit local plane using ransac
  // nr_valid_grid = fit();
  nr_valid_grid = fit_in_order();
  // std::cout << "# of valid plane geometry (fitting): " << nr_valid_grid <<
  // std::endl;
  // smooth plane using neighborhood information:
  for (iter = 0; iter < _param.nr_smooth_iter; ++iter) {
    nr_valid_grid = smooth();
  }

  for (r = 0; r < _param.nr_grids_coarse; ++r) {
    for (c = 0; c < _param.nr_grids_coarse; ++c) {
      if ((*_vg_coarse)(r, c).empty()) {
        _ground_planes[r][c].force_invalid();
      }
    }
  }

  // std::cout << "# of valid plane geometry (smooth): " << nr_valid_grid <<
  // std::endl;
  // compute point to ground distance
  compute_signed_ground_height(point_cloud, height_above_ground, nr_points,
                               nr_point_elements);
  return true;
}

const char *PlaneFitGroundDetector::get_label() const { return _labels; }

const VoxelGridXY<float> *PlaneFitGroundDetector::get_grid() const {
  return _vg_coarse;
}

const GroundPlaneLiDAR *PlaneFitGroundDetector::get_ground_plane(int r,
                                                                 int c) const {
  assert(r >= 0 && r < (int)_param.nr_grids_coarse);
  assert(c >= 0 && c < (int)_param.nr_grids_coarse);
  return _ground_planes != NULL ? _ground_planes[r] + c : NULL;
}

const unsigned int PlaneFitGroundDetector::get_grid_dim_x() const {
  return _vg_coarse->nr_voxel_x();
}

const unsigned int PlaneFitGroundDetector::get_grid_dim_y() const {
  return _vg_coarse->nr_voxel_y();
}

float PlaneFitGroundDetector::get_unknown_height() { return (FLT_MAX); }

PlaneFitPointCandIndices **PlaneFitGroundDetector::get_candis() const {
  return _local_candis;
}

void i_plane_eucli_to_spher(const GroundPlaneLiDAR &src,
                            GroundPlaneSpherical &dst) {
  if (src.is_valid() == false) {
    dst.force_invalid();
  } else {
    GroundPlaneLiDAR p = src;
    assert(p.params[2] != 0 || p.params[1] != 0);
    p.force_unit_norm();  // to be safe
    p.force_positive_normal_z();
    dst.theta = i_acos(p.params[0]);
    dst.phi = i_atan2(p.params[1], p.params[2]);
    dst.d = p.params[3];
    dst.set_nr_support(src.get_nr_support());
    dst.set_status(src.get_status());
  }
}

void i_plane_spher_to_eucli(const GroundPlaneSpherical &src,
                            GroundPlaneLiDAR &dst) {
  if (src.is_valid() == false) {
    dst.force_invalid();
  } else {
    // assume positive nz;
    // ny = nz * ny_over_nz;
    // nx * nx + ny * ny + nz * nz = 1.0;
    // nz^2 + nz^2*ny_over_nz^2 = 1 - nx^2;
    // nz^2(1 + ny_over_nz^2) = 1 - nx^2;
    // nz = sqrt((1-nx^2)/(1 + ny_over_nz^2))
    // nz is positive, guaranteed
    float nx = i_cos(src.theta);
    float ny_over_nz = i_tan(src.phi);
    float nz = i_sqrt((1 - nx * nx) / (1 + ny_over_nz * ny_over_nz));
    float ny = nz * ny_over_nz;
    dst.params[0] = nx;
    dst.params[1] = ny;
    dst.params[2] = nz;
    dst.params[3] = src.d;
    dst.set_nr_support(src.get_nr_support());
    dst.set_status(src.get_status());
  }
}
} /*namespace idl*/

/*
int PlaneFitGroundDetector::compare_z_16(const float* point_cloud,
const std::vector<int>& indices,
const float z_values[16],
unsigned int nr_points,
unsigned int nr_point_element,
PlaneFitPointCandIndices& candi) {
int pos, nr_candis = 0;
unsigned int i, nr_contradi = 0;
float z, delta_z;
for (i = 0; i < indices.size(); ++i) {
pos = indices[i];
assert(pos < (int)nr_points);
// requires the Z element to be in the third position, i.e., after X, Y
z = (point_cloud + (pos * nr_point_element))[2];
if (z < _param.sample_region_z_lower || z >= _param.sample_region_z_upper) {
continue;
}
// 0
delta_z = i_abs(z_values[0] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 1
delta_z = i_abs(z_values[1] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 2
delta_z = i_abs(z_values[2] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 3
delta_z = i_abs(z_values[3] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 4
delta_z = i_abs(z_values[4] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 5
delta_z = i_abs(z_values[5] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 6
delta_z = i_abs(z_values[6] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 7
delta_z = i_abs(z_values[7] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 8
delta_z = i_abs(z_values[8] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 9
delta_z = i_abs(z_values[9] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 10
delta_z = i_abs(z_values[10] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 11
delta_z = i_abs(z_values[11] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 12
delta_z = i_abs(z_values[12] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 13
delta_z = i_abs(z_values[13] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 14
delta_z = i_abs(z_values[14] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 15
delta_z = i_abs(z_values[15] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi <= _param.nr_z_comp_fail_threshold) {
_labels[pos] = 1;
candi.push_index(pos);
nr_candis++;
}
}
return nr_candis;
}

int PlaneFitGroundDetector::compare_z_32(const float* point_cloud,
const std::vector<int>& indices,
const float z_values[32],
unsigned int nr_points,
unsigned int nr_point_element,
PlaneFitPointCandIndices& candi) {
int pos, nr_candis = 0;
unsigned int i, nr_contradi = 0;
float z, delta_z;
for (i = 0; i < indices.size(); ++i) {
pos = indices[i];
assert(pos < (int)nr_points);
// requires the Z element to be in the third position, i.e., after X, Y
z = (point_cloud + (pos * nr_point_element))[2];
if (z < _param.sample_region_z_lower || z >= _param.sample_region_z_upper) {
continue;
}
// 0
delta_z = i_abs(z_values[0] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 1
delta_z = i_abs(z_values[1] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 2
delta_z = i_abs(z_values[2] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 3
delta_z = i_abs(z_values[3] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 4
delta_z = i_abs(z_values[4] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 5
delta_z = i_abs(z_values[5] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 6
delta_z = i_abs(z_values[6] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 7
delta_z = i_abs(z_values[7] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 8
delta_z = i_abs(z_values[8] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 9
delta_z = i_abs(z_values[9] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 10
delta_z = i_abs(z_values[10] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 11
delta_z = i_abs(z_values[11] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 12
delta_z = i_abs(z_values[12] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 13
delta_z = i_abs(z_values[13] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 14
delta_z = i_abs(z_values[14] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 15
delta_z = i_abs(z_values[15] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 16
delta_z = i_abs(z_values[16] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 17
delta_z = i_abs(z_values[17] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 18
delta_z = i_abs(z_values[18] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 19
delta_z = i_abs(z_values[19] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 20
delta_z = i_abs(z_values[20] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 21
delta_z = i_abs(z_values[21] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 22
delta_z = i_abs(z_values[22] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 23
delta_z = i_abs(z_values[23] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 24
delta_z = i_abs(z_values[24] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 25
delta_z = i_abs(z_values[25] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 26
delta_z = i_abs(z_values[26] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 27
delta_z = i_abs(z_values[27] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi > _param.nr_z_comp_fail_threshold) {
continue;
}
// 28
delta_z = i_abs(z_values[28] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 29
delta_z = i_abs(z_values[29] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 30
delta_z = i_abs(z_values[30] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
// 31
delta_z = i_abs(z_values[31] - z);
if (delta_z > _param.planefit_filter_threshold) {
nr_contradi++;
}
if (nr_contradi <= _param.nr_z_comp_fail_threshold) {
_labels[pos] = 1;
candi.push_index(pos);
nr_candis++;
}
}
return nr_candis;
}
*/
