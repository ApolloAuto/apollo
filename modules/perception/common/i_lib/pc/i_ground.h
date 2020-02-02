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
#pragma once

#include <utility>
#include <vector>

#include "modules/perception/common/i_lib/core/i_blas.h"
#include "modules/perception/common/i_lib/core/i_rand.h"
#include "modules/perception/common/i_lib/geometry/i_plane.h"
#include "modules/perception/common/i_lib/pc/i_struct_s.h"

namespace apollo {
namespace perception {
namespace common {
struct GroundPlaneLiDAR {
  GroundPlaneLiDAR() {
    IZero4(params);
    nr_support = 0;
    is_detected = false;
  }

  GroundPlaneLiDAR &operator=(const GroundPlaneLiDAR &pi) {
    ICopy4(pi.params, this->params);
    this->nr_support = pi.GetNrSupport();
    this->is_detected = pi.GetStatus();
    return (*this);
  }

  float GetDegreeNormalToX() const {
    float normal[3];
    ICopy3(params, normal);
    if (normal[0] < 0) {
      INeg3(normal);
    }
    // normalize:
    IScale3(normal, IRec(ISqrt(ISquaresum3(normal))));
    return IRadiansToDegree(IAcos(normal[0]));
  }

  float get_degree_normal_to_y() const {
    float normal[3];
    ICopy3(params, normal);
    if (normal[1] < 0) {
      INeg3(normal);
    }
    // normalize:
    IScale3(normal, IRec(ISqrt(ISquaresum3(normal))));
    return IRadiansToDegree(IAcos(normal[1]));
  }

  float GetDegreeNormalToZ() const {
    float normal[3];
    ICopy3(params, normal);
    if (normal[2] < 0) {
      INeg3(normal);
    }
    // normalize:
    IScale3(normal, IRec(ISqrt(ISquaresum3(normal))));
    return IRadiansToDegree(IAcos(normal[2]));
  }

  void ForcePositiveNormalX() {
    if (params[0] < 0) {
      INeg4(params);
    }
  }

  void ForcePositiveNormalY() {
    if (params[1] < 0) {
      INeg4(params);
    }
  }

  void ForcePositiveNormalZ() {
    if (params[2] < 0) {
      INeg4(params);
    }
  }

  void ForceInvalid() {
    IZero4(params);
    nr_support = 0;
  }

  void ForceUnitNorm() {
    float norm = IL2Norm3(params);
    IScale4(params, IRec(norm));
  }

  bool IsValid() const {
    if (nr_support && (params[0] != 0 || params[1] != 0 || params[2] != 0)) {
      return true;
    }
    return false;
  }

  int GetNrSupport() const { return nr_support; }

  void SetNrSupport(int nr) { nr_support = nr; }

  bool GetStatus() const { return is_detected; }

  void SetStatus(bool flag) { is_detected = flag; }

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

  GroundPlaneSpherical &operator=(const GroundPlaneSpherical &pi) {
    this->theta = pi.theta;
    this->phi = pi.phi;
    this->d = pi.d;
    this->nr_support = pi.GetNrSupport();
    this->is_detected = pi.GetStatus();
    return (*this);
  }

  bool IsValid() const { return (nr_support > 0); }

  void ForceInvalid() {
    theta = phi = d = 0;
    nr_support = 0;
  }

  int GetNrSupport() const { return nr_support; }

  void SetNrSupport(int nr) { nr_support = nr; }

  bool GetStatus() const { return is_detected; }

  void SetStatus(bool flag) { is_detected = flag; }

  float theta;
  float phi;
  float d;

 private:
  bool is_detected;
  int nr_support;
};

struct PlaneFitGroundDetectorParam {
  PlaneFitGroundDetectorParam() { SetDefault(); }
  bool Validate() const;
  void SetDefault();
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
  void Reserve(unsigned int size) { indices.reserve(size); }
  void Clear() { indices.clear(); }
  inline void PushIndex(int i) { indices.push_back(i); }
  int Prune(unsigned int min_nr_samples, unsigned int max_nr_samples);
  unsigned int Size() const {
    return static_cast<unsigned int>(indices.size());
  }
  int &operator[](unsigned int i) {
    assert(i >= 0 && i < indices.size());
    return indices.at(i);
  }
  int &operator[](int i) {
    assert(i >= 0 && i < static_cast<int>(indices.size()));
    return indices.at(i);
  }
  const int &operator[](unsigned int i) const {
    assert(i >= 0 && i < indices.size());
    return indices.at(i);
  }
  const int &operator[](int i) const {
    assert(i >= 0 && i < static_cast<int>(indices.size()));
    return indices.at(i);
  }
  std::vector<int> indices;
  int random_seed;
};

void IPlaneEucliToSpher(const GroundPlaneLiDAR &src, GroundPlaneSpherical *dst);

void IPlaneSpherToEucli(const GroundPlaneSpherical &src, GroundPlaneLiDAR *dst);

class BaseGroundDetector {
 public:
  explicit BaseGroundDetector(const PlaneFitGroundDetectorParam &param)
      : param_(param) {}
  virtual ~BaseGroundDetector() {}
  virtual bool Detect(const float *point_cloud, float *height_above_ground,
                      unsigned int nr_points,
                      unsigned int nr_point_elements) = 0;

 protected:
  const PlaneFitGroundDetectorParam &param_;
};

class PlaneFitGroundDetector : public BaseGroundDetector {
  static const int dim_point_ = 3;

 public:
  explicit PlaneFitGroundDetector(const PlaneFitGroundDetectorParam &param);
  ~PlaneFitGroundDetector();
  bool Init();
  bool Detect(const float *point_cloud, float *height_above_ground,
              unsigned int nr_points, unsigned int nr_point_elements);
  const char *GetLabel() const;
  const VoxelGridXY<float> *GetGrid() const;
  const GroundPlaneLiDAR *GetGroundPlane(int r, int c) const;
  unsigned int GetGridDimX() const;
  unsigned int GetGridDimY() const;
  float GetUnknownHeight();
  PlaneFitPointCandIndices **GetCandis() const;

 protected:
  void CleanUp();
  void InitOrderTable(const VoxelGridXY<float> *vg, std::pair<int, int> *order);
  int Fit();
  int FitLine(unsigned int r);
  int FitGrid(const float *point_cloud, PlaneFitPointCandIndices *candi,
              GroundPlaneLiDAR *groundplane, unsigned int nr_points,
              unsigned int nr_point_element, float dist_thre);
  int FitInOrder();
  int FilterCandidates(int r, int c, const float *point_cloud,
                       PlaneFitPointCandIndices *candi,
                       std::vector<std::pair<int, int> > *neighbors,
                       unsigned int nr_point_element);
  int FitGridWithNeighbors(int r, int c, const float *point_cloud,
                           GroundPlaneLiDAR *groundplane,
                           unsigned int nr_points,
                           unsigned int nr_point_element, float dist_thre);
  void GetNeighbors(int r, int c, int rows, int cols,
                    std::vector<std::pair<int, int> > *neighbors);
  float CalculateAngleDist(const GroundPlaneLiDAR &plane,
                           const std::vector<std::pair<int, int> > &neighbors);
  int Filter();
  int FilterLine(unsigned int r);
  int FilterGrid(const Voxel<float> &vg, const float *point_cloud,
                 PlaneFitPointCandIndices *candi, unsigned int nr_points,
                 unsigned int nr_point_element);
  int Smooth();
  int SmoothLine(unsigned int up, unsigned int r, unsigned int dn);
  int CompleteGrid(const GroundPlaneSpherical &lt,
                   const GroundPlaneSpherical &rt,
                   const GroundPlaneSpherical &up,
                   const GroundPlaneSpherical &dn, GroundPlaneSpherical *gp);
  int SmoothGrid(const GroundPlaneSpherical &g, const GroundPlaneSpherical &lt,
                 const GroundPlaneSpherical &rt, const GroundPlaneSpherical &up,
                 const GroundPlaneSpherical &dn, GroundPlaneSpherical *gp);
  int CompareZ(const float *point_cloud, const std::vector<int> &point_indices,
               const float *z_values, PlaneFitPointCandIndices *candi,
               unsigned int nr_points, unsigned int nr_point_element,
               unsigned int nr_compares);
  void ComputeAdaptiveThreshold();
  void ComputeSignedGroundHeight(const float *point_cloud,
                                 float *height_above_ground,
                                 unsigned int nr_points,
                                 unsigned int nr_point_elements);
  void ComputeSignedGroundHeightLine(const float *point_cloud,
                                     const GroundPlaneLiDAR *up,
                                     const GroundPlaneLiDAR *cn,
                                     const GroundPlaneLiDAR *dn,
                                     float *height_above_ground, unsigned int r,
                                     unsigned int nr_points,
                                     unsigned int nr_point_elements);

 protected:
  VoxelGridXY<float> *vg_fine_;
  VoxelGridXY<float> *vg_coarse_;
  GroundPlaneLiDAR **ground_planes_;
  GroundPlaneSpherical **ground_planes_sphe_;
  PlaneFitPointCandIndices **local_candis_;
  std::pair<float, bool> **ground_z_;
  float **pf_thresholds_;
  unsigned int *map_fine_to_coarse_;
  char *labels_;
  float *sampled_z_values_;
  float *pf_threeds_;
  int *sampled_indices_;
  std::pair<int, int> *order_table_;
};

}  // namespace common
}  // namespace perception
}  // namespace apollo
