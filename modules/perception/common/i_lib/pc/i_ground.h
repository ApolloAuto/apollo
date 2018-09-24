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
#ifndef I_LIB_PC_I_GROUND_H
#define I_LIB_PC_I_GROUND_H

#include <vector>
#include "../core/i_blas.h"
#include "../core/i_rand.h"
#include "../geometry/i_plane.h"
#include "i_struct_s.h"

namespace idl {
struct GroundPlaneLiDAR {
  GroundPlaneLiDAR() {
    i_zero4(params);
    nr_support = 0;
    is_detected = false;
  }

  GroundPlaneLiDAR& operator=(const GroundPlaneLiDAR& pi) {
    i_copy4(pi.params, this->params);
    this->nr_support = pi.get_nr_support();
    this->is_detected = pi.get_status();
    return (*this);
  }

  float get_degree_normal_to_x() const {
    float normal[3];
    i_copy3(params, normal);
    if (normal[0] < 0) {
      i_neg3(normal);
    }
    // normalize:
    i_scale3(normal, i_rec(i_sqrt(i_squaresum3(normal))));
    return i_radians_to_degree(i_acos(normal[0]));
  }

  float get_degree_normal_to_y() const {
    float normal[3];
    i_copy3(params, normal);
    if (normal[1] < 0) {
      i_neg3(normal);
    }
    // normalize:
    i_scale3(normal, i_rec(i_sqrt(i_squaresum3(normal))));
    return i_radians_to_degree(i_acos(normal[1]));
  }

  float get_degree_normal_to_z() const {
    float normal[3];
    i_copy3(params, normal);
    if (normal[2] < 0) {
      i_neg3(normal);
    }
    // normalize:
    i_scale3(normal, i_rec(i_sqrt(i_squaresum3(normal))));
    return i_radians_to_degree(i_acos(normal[2]));
  }

  void force_positive_normal_x() {
    if (params[0] < 0) {
      i_neg4(params);
    }
  }

  void force_positive_normal_y() {
    if (params[1] < 0) {
      i_neg4(params);
    }
  }

  void force_positive_normal_z() {
    if (params[2] < 0) {
      i_neg4(params);
    }
  }

  void force_invalid() {
    i_zero4(params);
    nr_support = 0;
  }

  void force_unit_norm() {
    float norm = i_l2_norm3(params);
    i_scale4(params, i_rec(norm));
  }

  bool is_valid() const {
    if (nr_support && (params[0] != 0 || params[1] != 0 || params[2] != 0)) {
      return true;
    } else {
      return false;
    }
  }

  int get_nr_support() const { return nr_support; }

  void set_nr_support(int nr) { nr_support = nr; }

  bool get_status() const { return is_detected; }

  void set_status(bool flag) { is_detected = flag; }

  float params[4];

 private:
  bool is_detected;
  int nr_support;
};

struct GroundPlaneSpherical {
  GroundPlaneSpherical() {
    theta = phi = d = 0;
    nr_support = 0;
    is_detected = false;
  }

  GroundPlaneSpherical& operator=(const GroundPlaneSpherical& pi) {
    this->theta = pi.theta;
    this->phi = pi.phi;
    this->d = pi.d;
    this->nr_support = pi.get_nr_support();
    this->is_detected = pi.get_status();
    return (*this);
  }

  bool is_valid() const { return (nr_support > 0); }

  void force_invalid() {
    theta = phi = d = 0;
    nr_support = 0;
  }

  int get_nr_support() const { return nr_support; }

  void set_nr_support(int nr) { nr_support = nr; }

  bool get_status() const { return is_detected; }

  void set_status(bool flag) { is_detected = flag; }

  float theta;
  float phi;
  float d;

 private:
  bool is_detected;
  int nr_support;
};

struct PlaneFitGroundDetectorParam {
  PlaneFitGroundDetectorParam() { set_default(); }
  bool validate() const;
  void set_default();
  unsigned int nr_points_max;
  unsigned int nr_grids_fine;
  unsigned int nr_grids_coarse;
  unsigned int nr_z_comp_candis;
  unsigned int nr_z_comp_fail_threshold;
  unsigned int nr_samples_min_threshold;
  unsigned int nr_inliers_min_threshold;
  unsigned int nr_samples_max_threshold;
  float sample_region_z_lower;
  float sample_region_z_upper;
  float roi_region_rad_x;
  float roi_region_rad_y;
  float roi_region_rad_z;
  float roi_near_rad;
  float planefit_dist_threshold_near;
  float planefit_dist_threshold_far;
  float planefit_filter_threshold;
  float planefit_orien_threshold;
  float termi_inlier_percen_threshold;
  float candidate_filter_threshold;
  int nr_ransac_iter_threshold;
  int nr_smooth_iter;
};

struct PlaneFitPointCandIndices {
  PlaneFitPointCandIndices() { random_seed = I_DEFAULT_SEED; }
  void reserve(unsigned int size) { indices.reserve(size); }
  void clear() { indices.clear(); }
  inline void push_index(int i) { indices.push_back(i); }
  int prune(unsigned int min_nr_samples, unsigned int max_nr_samples);
  unsigned int size() const { return indices.size(); }
  int& operator[](unsigned int i) {
    assert(i >= 0 && i < indices.size());
    return indices.at(i);
  }
  int& operator[](int i) {
    assert(i >= 0 && i < (int)indices.size());
    return indices.at(i);
  }
  const int& operator[](unsigned int i) const {
    assert(i >= 0 && i < indices.size());
    return indices.at(i);
  }
  const int& operator[](int i) const {
    assert(i >= 0 && i < (int)indices.size());
    return indices.at(i);
  }
  std::vector<int> indices;
  int random_seed;
};

void i_plane_eucli_to_spher(const GroundPlaneLiDAR& src,
                            GroundPlaneSpherical& dst);

void i_plane_spher_to_eucli(const GroundPlaneSpherical& src,
                            GroundPlaneLiDAR& dst);

class BaseGroundDetector {
 public:
  explicit BaseGroundDetector(const PlaneFitGroundDetectorParam& param)
      : _param(param) {}
  virtual ~BaseGroundDetector() {}
  virtual bool detect(const float* point_cloud, float* height_above_ground,
                      unsigned int nr_points,
                      unsigned int nr_point_elements) = 0;

 protected:
  const PlaneFitGroundDetectorParam& _param;
};

class PlaneFitGroundDetector : public BaseGroundDetector {
  static const int _dim_point = 3;

 public:
  explicit PlaneFitGroundDetector(const PlaneFitGroundDetectorParam& param);
  ~PlaneFitGroundDetector();
  bool init();
  bool detect(const float* point_cloud, float* height_above_ground,
              unsigned int nr_points, unsigned int nr_point_elements);
  const char* get_label() const;
  const VoxelGridXY<float>* get_grid() const;
  const GroundPlaneLiDAR* get_ground_plane(int r, int c) const;
  const unsigned int get_grid_dim_x() const;
  const unsigned int get_grid_dim_y() const;
  float get_unknown_height();
  PlaneFitPointCandIndices** get_candis() const;

 protected:
  void cleanup();
  void init_order_table(const VoxelGridXY<float>* vg,
                        std::pair<int, int>* order);
  int fit();
  int fit_line(unsigned int r);
  int fit_grid(const float* point_cloud, PlaneFitPointCandIndices& candi,
               GroundPlaneLiDAR& groundplane, unsigned int nr_points,
               unsigned int nr_point_element, float dist_thre);
  int fit_in_order();
  int filter_candidates(int r, int c, const float* point_cloud,
                        PlaneFitPointCandIndices& candi,
                        std::vector<std::pair<int, int>>& neighbors,
                        unsigned int nr_point_element);
  int fit_grid_with_neighbors(int r, int c, const float* point_cloud,
                              GroundPlaneLiDAR& groundplane,
                              unsigned int nr_points,
                              unsigned int nr_point_element, float dist_thre);
  void get_neighbors(int r, int c, int rows, int cols,
                     std::vector<std::pair<int, int>>& neighbors);
  float calculate_angle_dist(const GroundPlaneLiDAR& plane,
                             const std::vector<std::pair<int, int>>& neighbors);
  int filter();
  int filter_line(unsigned int r);
  int filter_grid(const Voxel<float>& vg, const float* point_cloud,
                  PlaneFitPointCandIndices& candi, unsigned int nr_points,
                  unsigned int nr_point_element);
  int smooth();
  int smooth_line(unsigned int up, unsigned int r, unsigned int dn);
  int complete_grid(  // const GroundPlaneSpherical& g,
      const GroundPlaneSpherical& lt, const GroundPlaneSpherical& rt,
      const GroundPlaneSpherical& up, const GroundPlaneSpherical& dn,
      GroundPlaneSpherical& gp);
  int smooth_grid(const GroundPlaneSpherical& g, const GroundPlaneSpherical& lt,
                  const GroundPlaneSpherical& rt,
                  const GroundPlaneSpherical& up,
                  const GroundPlaneSpherical& dn, GroundPlaneSpherical& gp);
  int compare_z(const float* point_cloud, const std::vector<int>& point_indices,
                const float* z_values, PlaneFitPointCandIndices& candi,
                unsigned int nr_points, unsigned int nr_point_element,
                unsigned int nr_compares);
  void compute_adaptive_threshold();
  void compute_signed_ground_height(const float* point_cloud,
                                    float* height_above_ground,
                                    unsigned int nr_points,
                                    unsigned int nr_point_elements);
  void compute_signed_ground_height_line(const float* point_cloud,
                                         const GroundPlaneLiDAR* up,
                                         const GroundPlaneLiDAR* cn,
                                         const GroundPlaneLiDAR* dn,
                                         float* height_above_ground,
                                         unsigned int r, unsigned int nr_points,
                                         unsigned int nr_point_elements);

 protected:
  VoxelGridXY<float>* _vg_fine;
  VoxelGridXY<float>* _vg_coarse;
  // GroundPlaneLiDAR _global_plane;
  GroundPlaneLiDAR** _ground_planes;
  GroundPlaneSpherical** _ground_planes_sphe;
  PlaneFitPointCandIndices** _local_candis;
  std::pair<float, bool>** _ground_z;
  float** _pf_thresholds;
  unsigned int* _map_fine_to_coarse;
  char* _labels;
  float* _sampled_z_values;
  float* _pf_threeds;
  int* _sampled_indices;
  std::pair<int, int>* _order_table;
};

} /*namespace idl*/

#endif
