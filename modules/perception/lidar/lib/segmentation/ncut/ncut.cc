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

#include <omp.h>
#include <algorithm>
#include <ctime>
#include <functional>
#include <queue>
#include <stack>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/lidar/lib/segmentation/ncut/ncut.h"
/*#include "lib/graph_felzenszwalb/filter.h"
#include "lib/graph_felzenszwalb/image.h"
#include "lib/graph_felzenszwalb/misc.h"
#include "lib/graph_felzenszwalb/segment_image.h"*/

namespace apollo {
namespace perception {
namespace lidar {

namespace {
const int OBSTACLE_MINIMUM_NUM_POINTS = 50;
}

using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::GetProtoFromFile;
using Eigen::MatrixXf;

NCut::NCut() {}
NCut::~NCut() { ADEBUG << "NCut destructor done"; }

bool NCut::Init(const NCutParam &param) {
  if (!Configure(param)) {
    AERROR << "failed to load ncut config.";
    return false;
  }

  _classifier.reset(new LRClassifier);
  if (!_classifier->init()) {
    AERROR << "failed to init FrameClassifierPipeline.";
    return false;
  }
  return true;
}

bool NCut::Configure(const NCutParam &ncut_param_) {
  _grid_radius = ncut_param_.grid_radius();
  _connect_radius = ncut_param_.connect_radius();
  _super_pixel_cell_size = ncut_param_.super_pixel_cell_size();
  _num_cuts = ncut_param_.num_cuts();
  _ncuts_stop_threshold = ncut_param_.ncuts_stop_threshold();
  _ncuts_enable_classifier_threshold =
      ncut_param_.ncuts_enable_classifier_threshold();
  _sigma_space = ncut_param_.sigma_space();
  _sigma_feature = ncut_param_.sigma_feature();
  _skeleton_cell_size = ncut_param_.skeleton_cell_size();
  _patch_size = ncut_param_.patch_size();
  _overlap_factor = ncut_param_.overlap_factor();
  _felzenszwalb_sigma = ncut_param_.felzenszwalb_sigma();
  _felzenszwalb_k = ncut_param_.felzenszwalb_k();
  _felzenszwalb_min_size = ncut_param_.felzenszwalb_min_size();
  return true;
}

void NCut::Segment(base::PointFCloudConstPtr cloud) {
  double start_t = omp_get_wtime();
  // .0 clear everything
  _segment_pids.clear();
  _segment_labels.clear();
  _segment_bbox.clear();
  _outlier_pids.clear();
  _cluster_points.clear();
  _cluster_bounding_box.clear();
  _cluster_skeleton_points.clear();
  _cluster_skeleton_features.clear();
  std::vector<int> pids(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    pids[i] = static_cast<int>(i);
  }
  _cloud_obstacles = base::PointFCloudPtr(new base::PointFCloud(*cloud, pids));
#ifdef DEBUG_NCUT
  ADEBUG << "segment enter ... input cloud size " << cloud->size();
// visualize_points(pids);
#endif
  // .1 super pixels
  // super_pixels_felzenszwalb(cloud, _grid_radius, _super_pixel_cell_size,
  // _cluster_points);
  SuperPixelsFloodFill(cloud, _grid_radius, _super_pixel_cell_size,
                       &_cluster_points);
#ifdef DEBUG_NCUT
  // visualize_segments_from_points(_cluster_points);
  ADEBUG << "super pixels " << _cluster_points.size();
#endif
  ADEBUG << "super pixels done " << omp_get_wtime() - start_t;
  start_t = omp_get_wtime();
  // .2 precompute skeleton and bbox
  PrecomputeAllSkeletonAndBbox();
  // LOG_DEBUG << "precompute skeleton and bbox done " << omp_get_wtime() -
  // start_t;
  start_t = omp_get_wtime();
  // .3 grach cut
  std::vector<std::vector<int>> segment_clusters;
  std::vector<std::string> segment_labels;
  NormalizedCut(_ncuts_stop_threshold, true, &segment_clusters,
                &segment_labels);
  ADEBUG << "normalized_cut done, #segments " << segment_clusters.size()
         << ", time: " << omp_get_wtime() - start_t;
  start_t = omp_get_wtime();
  // .4 _segment_pids;
  for (size_t i = 0; i < segment_clusters.size(); ++i) {
    std::vector<int> pids;
    GetClustersPids(segment_clusters[i], &pids);
    if (pids.size() > 0) {
      _segment_pids.push_back(pids);
      _segment_labels.push_back(segment_labels[i]);
      NcutBoundingBox box;
      GetComponentBoundingBox(segment_clusters[i], &box);
      _segment_bbox.push_back(box);
    }
  }
}

void NCut::SuperPixelsFloodFill(base::PointFCloudConstPtr cloud, float radius,
                                float cell_size,
                                std::vector<std::vector<int>> *super_pixels) {
  FloodFill ff_grid(radius, cell_size);
  std::vector<int> num_cells_per_components;
  ff_grid.GetSegments(cloud, super_pixels, &num_cells_per_components);
}

void NCut::PrecomputeAllSkeletonAndBbox() {
  const int num_clusters = static_cast<int>(_cluster_points.size());
  _cluster_skeleton_points.resize(num_clusters);
  _cluster_skeleton_features.resize(num_clusters);
  _cluster_bounding_box.resize(num_clusters);
  _cluster_labels.resize(num_clusters, "unknown");
  // skeleton
  _ff_feature_grid.SetGridRadius(_grid_radius);
  _ff_feature_grid.SetCellSize(_skeleton_cell_size);
  _ff_feature_grid.BuildGrid(_cloud_obstacles);
  std::vector<gridIndex> dummy_indices;
  BuildAverageHeightMap(_cloud_obstacles, _ff_feature_grid, &_cv_feature_map,
                        &dummy_indices);

  // build_maximum_height_map(_cloud_obstacles, _ff_feature_grid,
  // _cv_feature_map, dummy_indices);
  for (int i = 0; i < num_clusters; ++i) {
    SampleByGrid(_cluster_points[i], &_cluster_skeleton_points[i],
                 &_cluster_skeleton_features[i]);
    _cluster_bounding_box[i] = ComputeClusterBoundingBox(_cluster_points[i]);
  }
  // cluster label
  for (size_t i = 0; i < _cluster_points.size(); ++i) {
    base::PointFCloudPtr pc = base::PointFCloudPtr(
        new base::PointFCloud(*_cloud_obstacles, _cluster_points[i]));
    _cluster_labels[i] = GetPcLabel(pc);
  }
}

void NCut::BuildAverageHeightMap(
    base::PointFCloudConstPtr cloud, const FloodFill &ff_map,
    cv::Mat *cv_height_map_in, std::vector<gridIndex> *point_pixel_indices_in) {
  cv::Mat &cv_height_map = *cv_height_map_in;
  std::vector<gridIndex> &point_pixel_indices = *point_pixel_indices_in;
  const int num_points = static_cast<int>(cloud->size());
  cv::Mat cv_img =
      cv::Mat::zeros(ff_map.GetNumRows(), ff_map.GetNumCols(), CV_32F);
  std::vector<std::vector<int>> counts;
  counts.resize(cv_img.rows);
  for (size_t i = 0; i < counts.size(); ++i) {
    counts[i].resize(cv_img.cols, 0);
  }
  point_pixel_indices.resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    int irow = -1;
    int jcol = -1;
    base::PointF pt = (*cloud)[i];
    if (ff_map.Pos2d(pt.x, pt.y, &irow, &jcol)) {
      cv_img.at<float>(irow, jcol) += pt.z;
      counts[irow][jcol] += 1;
      point_pixel_indices[i].irow = irow;
      point_pixel_indices[i].jcol = jcol;
    } else {
      point_pixel_indices[i].irow = -1;
      point_pixel_indices[i].jcol = -1;
    }
  }
  for (int i = 0; i < cv_img.rows; ++i) {
    for (int j = 0; j < cv_img.cols; ++j) {
      if (counts[i][j] > 0) {
        cv_img.at<float>(i, j) /= static_cast<float>(counts[i][j]);
      }
    }
  }
  cv::Mat cv_height_map_copy;
  cv::GaussianBlur(cv_img, cv_height_map_copy, cv::Size(3, 3), 0, 0);
  cv::normalize(cv_height_map_copy, cv_height_map, 0, 255, cv::NORM_MINMAX,
                CV_8UC1);
}

void NCut::SampleByGrid(const std::vector<int> &point_gids,
                        MatrixXf *skeleton_coords_in,
                        MatrixXf *skeleton_feature_in) {
  MatrixXf &skeleton_coords = *skeleton_coords_in;
  MatrixXf &skeleton_feature = *skeleton_feature_in;
  FloodFill sampler(_grid_radius, _skeleton_cell_size);
  base::PointFCloudPtr pc = base::PointFCloudPtr(
      new base::PointFCloud(*_cloud_obstacles, point_gids));
  sampler.BuildGrid(pc);
  std::unordered_map<int, std::pair<base::PointF, float>> centroids;
  const std::vector<int> &point_idx = sampler.GetPointIdxInGrid();
  std::unordered_map<int, std::pair<base::PointF, float>>::iterator it;
  for (size_t i = 0; i < point_idx.size(); ++i) {
    it = centroids.find(point_idx[i]);
    if (it != centroids.end()) {
      it->second.first.x += (*pc)[i].x;
      it->second.first.y += (*pc)[i].y;
      it->second.first.z += (*pc)[i].z;
      it->second.second += 1.f;
    } else {
      centroids[point_idx[i]] = std::make_pair((*pc)[i], 1.f);
    }
  }
  int num_skeletons = static_cast<int>(centroids.size());
  skeleton_coords.resize(num_skeletons, 3);
  int p = 0;
  for (it = centroids.begin(); it != centroids.end(); ++it) {
    skeleton_coords.coeffRef(p, 0) = it->second.first.x / it->second.second;
    skeleton_coords.coeffRef(p, 1) = it->second.first.y / it->second.second;
    skeleton_coords.coeffRef(p, 2) = it->second.first.z / it->second.second;
    p++;
  }
  GetPatchFeature(skeleton_coords, &skeleton_feature);
}

void NCut::GetPatchFeature(const MatrixXf &points, MatrixXf *features_in) {
  MatrixXf &features = *features_in;
  const int num_points = static_cast<int>(points.rows());
  const int dim = _patch_size * _patch_size;
  features.resize(num_points, dim);
  for (int i = 0; i < num_points; ++i) {
    // .1 patch
    int irow = 0;
    int jcol = 0;
    _ff_feature_grid.Pos2d(points.coeffRef(i, 0), points.coeffRef(i, 1), &irow,
                           &jcol);
    cv::Mat patch;
    cv::Point2f pt(static_cast<float>(irow), static_cast<float>(jcol));
    cv::getRectSubPix(_cv_feature_map, cv::Size(_patch_size, _patch_size), pt,
                      patch);
    // .2 maybe later i will add other features based on patch
    // .3 add to features
    assert(dim == patch.rows * patch.cols);
    int p = 0;
    for (int r = 0; r < patch.rows; ++r) {
      for (int c = 0; c < patch.cols; ++c) {
        float val = patch.at<float>(r, c);
        features.coeffRef(i, p++) =
            static_cast<float>((isnan(val) || isinf(val)) ? 1.e-50 : val);
        // features.coeffRef(i, p++) = patch.at<float>(r, c);
      }
    }
  }
}

NCut::NcutBoundingBox NCut::ComputeClusterBoundingBox(
    const std::vector<int> &point_gids) {
  // ! Note: do not perform rotation, so just some intuitive guess
  float x_max = -FLT_MAX;
  float y_max = -FLT_MAX;
  float z_max = -FLT_MAX;
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  for (size_t j = 0; j < point_gids.size(); ++j) {
    int pid = point_gids[j];
    x_min = std::min(x_min, (*_cloud_obstacles)[pid].x);
    x_max = std::max(x_max, (*_cloud_obstacles)[pid].x);
    y_min = std::min(y_min, (*_cloud_obstacles)[pid].y);
    y_max = std::max(y_max, (*_cloud_obstacles)[pid].y);
    z_min = std::min(z_min, (*_cloud_obstacles)[pid].z);
    z_max = std::max(z_max, (*_cloud_obstacles)[pid].z);
  }
  NcutBoundingBox box;
  std::get<0>(box) = x_min;
  std::get<1>(box) = x_max;
  std::get<2>(box) = y_min;
  std::get<3>(box) = y_max;
  std::get<4>(box) = z_min;
  std::get<5>(box) = z_max;
  return box;
}

std::string NCut::GetPcLabel(const base::PointFCloudPtr &cloud) {
  if (cloud->size() < OBSTACLE_MINIMUM_NUM_POINTS) {
    return "unknown";
  }
  base::PointFCloudPtr rot_cloud(new base::PointFCloud);
  base::OrientCloud(*cloud, rot_cloud.get(), true);
  std::string label;
  // Eigen::VectorXf response = _classifier->classify_and_get_label(rot_cloud,
  // &label);
  label = _classifier->GetLabel(rot_cloud);
  // ./obstacle_detection/classifier/frame_classifier_pipeline.cpp line 466
  if (label == "background") {
    label = "unknown";
  }
  return label;
}

void NCut::NormalizedCut(float ncuts_threshold, bool use_classifier,
                         std::vector<std::vector<int>> *segment_clusters_in,
                         std::vector<std::string> *segment_labels_in) {
  std::vector<std::vector<int>> &segment_clusters = *segment_clusters_in;
  std::vector<std::string> &segment_labels = *segment_labels_in;
  const int num_clusters = static_cast<int>(_cluster_points.size());
  if (num_clusters < 1) {
    return;
  }
  if (num_clusters == 1) {
    std::vector<int> tmp(1, 0);
    segment_clusters.push_back(tmp);
    segment_labels.push_back(_cluster_labels[0]);
    return;
  }
#ifdef DEBUG_NCUT
  LOG_DEBUG << "\n+++++++++++++++++++++++++++++ input " << num_clusters
            << " clusters +++++++++++++++++++++++++++\n";
// visualize_segments_from_cluster(_cluster_points);
#endif
  MatrixXf weights;
  ComputeSkeletonWeights(&weights);
  std::vector<int> *curr = new std::vector<int>(num_clusters);
  for (int i = 0; i < num_clusters; ++i) {
    (*curr)[i] = i;
  }
  std::stack<std::vector<int> *> job_stack;
  job_stack.push(curr);
  while (!job_stack.empty()) {
    curr = job_stack.top();
    job_stack.pop();
#ifdef DEBUG_NCUT
    LOG_DEBUG << "curr size " << curr->size();
// visualize_cluster(curr);
#endif
    std::string seg_label;
    if (curr->size() == 1) {
      segment_clusters.push_back(*curr);
      segment_labels.push_back(_cluster_labels[(*curr)[0]]);
    } else if (use_classifier && IsMovableObstacle(*curr, &seg_label)) {
      segment_clusters.push_back(*curr);
      segment_labels.push_back(seg_label);
#ifdef DEBUG_NCUT
      std::cout << "cluster: ";
      for (size_t i = 0; i < curr->size(); ++i) {
        std::cout << (*curr)[i] << " ";
      }
      std::cout << " as a segment (" << seg_label << ")" << std::endl;
#endif
    } else {
      std::vector<int> *seg1 = new std::vector<int>();
      std::vector<int> *seg2 = new std::vector<int>();
      MatrixXf my_weights(curr->size(), curr->size());
      for (size_t i = 0; i < curr->size(); ++i) {
        const int ci = curr->at(i);
        for (size_t j = 0; j < curr->size(); ++j) {
          const int cj = curr->at(j);
          my_weights.coeffRef(i, j) = weights.coeffRef(ci, cj);
        }
      }
      double cost = GetMinNcuts(my_weights, curr, seg1, seg2);
#ifdef DEBUG_NCUT
      LOG_DEBUG << "N cut cost is " << cost << ", seg1 size " << seg1->size()
                << ", seg2 size " << seg2->size();
      if (curr->size() < 50) {
        std::cout << "seg1: ";
        for (size_t i = 0; i < seg1->size(); ++i) {
          std::cout << (*seg1)[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "seg2: ";
        for (size_t i = 0; i < seg2->size(); ++i) {
          std::cout << (*seg2)[i] << " ";
        }
        std::cout << std::endl;
      }
// visualize_graph_cut(seg1, seg2);
#endif
      if (cost > ncuts_threshold || 0 == seg1->size() || 0 == seg2->size()) {
        std::vector<int> buffer;
        for (size_t i = 0; i < curr->size(); ++i) {
          const int cid = (*curr)[i];
          if (_cluster_labels[cid] != "unknown") {
#ifdef DEBUG_NCUT
            std::cout << "cluster: " << cid << " as a segment (hit, "
                      << _cluster_labels[cid] << ")" << std::endl;
#endif
            std::vector<int> tmp(1, cid);
            segment_clusters.push_back(tmp);
            segment_labels.push_back(_cluster_labels[cid]);
          } else {
            buffer.push_back(cid);
          }
        }
        if (buffer.size() > 0) {
          segment_clusters.push_back(buffer);
          segment_labels.push_back("unknown");
        }
#ifdef DEBUG_NCUT
        std::cout << "cluster: ";
        for (size_t i = 0; i < buffer.size(); ++i) {
          std::cout << buffer[i] << " ";
        }
        std::cout << " as a segment (hit, unknown)" << std::endl;
#endif
        delete seg1;
        delete seg2;
      } else {
        job_stack.push(seg1);
        job_stack.push(seg2);
      }
    }
#ifdef DEBUG_NCUT
    LOG_DEBUG << "============================================================="
                 "================";
#endif
    delete curr;
  }  // end of while
#ifdef DEBUG_NCUT
  std::cout << "graph cut return segments: " << std::endl;
  for (size_t i = 0; i < segment_clusters.size(); ++i) {
    std::cout << "seg " << i << ": ";
    for (size_t j = 0; j < segment_clusters[i].size(); ++j) {
      std::cout << segment_clusters[i][j] << " ";
    }
    std::cout << ": " << segment_labels[i] << std::endl;
  }
  LOG_DEBUG << "normalized_cut: " << segment_clusters.size()
            << " segments from " << _cluster_points.size() << " clusters";
// visualize_segments_from_cluster(segment_clusters);
#endif
}

void NCut::ComputeSkeletonWeights(Eigen::MatrixXf *weights_in) {
  Eigen::MatrixXf &weights = *weights_in;
  const int num_clusters = static_cast<int>(_cluster_points.size());
  const double hs2 = _sigma_space * _sigma_space;
  const double hf2 = _sigma_feature * _sigma_feature;
  const double radius2 = _connect_radius * _connect_radius;
  weights.resize(num_clusters, num_clusters);
  for (int i = 0; i < num_clusters; ++i) {
    weights.coeffRef(i, i) = 1.f;
    for (int j = i + 1; j < num_clusters; ++j) {
      float dist_point = FLT_MAX;
      float dist_feature = FLT_MAX;
      ComputeSquaredSkeletonDistance(
          _cluster_skeleton_points[i], _cluster_skeleton_features[i],
          _cluster_skeleton_points[j], _cluster_skeleton_features[j],
          &dist_point, &dist_feature);
      if (dist_point > radius2) {
        weights.coeffRef(i, j) = 0.f;
        weights.coeffRef(j, i) = 0.f;
      } else {
        weights.coeffRef(i, j) = static_cast<float>(exp(-dist_point / hs2) *
                                                    exp(-dist_feature / hf2));
        weights.coeffRef(j, i) = weights.coeffRef(i, j);
      }
    }
  }
}

float NCut::GetMinNcuts(const Eigen::MatrixXf &in_weights,
                        const std::vector<int> *in_clusters,
                        std::vector<int> *seg1, std::vector<int> *seg2) {
  // .0 initialization
  const int num_clusters = static_cast<int>(in_weights.rows());
  seg1->resize(num_clusters);
  seg2->resize(num_clusters);
  // .1 eigen decompostion
  Eigen::MatrixXf eigenvectors;
  LaplacianDecomposition(in_weights, &eigenvectors);
// std::cout << "\nweights: " << in_weights << std::endl;
// std::cout << "\neigv: " << eigenvectors << std::endl;
#ifdef DEBUG_NCUT
// std::cout << "weights:\n " << in_weights << std::endl << std::endl;
// std::cout << "eigen vectors:\n " << eigenvectors << std::endl << std::endl;
#endif
  // .2 search for best split
  const float minval = eigenvectors.col(1).minCoeff();
  const float maxval = eigenvectors.col(1).maxCoeff();
  const float increment = static_cast<float>(
      (maxval - minval) / (static_cast<float>(_num_cuts) + 1.0f));
  int num_seg1 = 0;
  int num_seg2 = 0;
  float opt_split = 0.0;
  float opt_cost = FLT_MAX;
  for (int i = 0; i < _num_cuts; ++i) {
    num_seg1 = 0;
    num_seg2 = 0;
    // .2.1 split
    float split =
        static_cast<float>(minval + static_cast<float>(i + 1) * increment);
    for (int j = 0; j < num_clusters; ++j) {
      if (eigenvectors.coeffRef(j, 1) > split) {
        (*seg1)[num_seg1++] = j;
      } else {
        (*seg2)[num_seg2++] = j;
      }
    }
    // .2.2 compute best normalized_cuts cost
    double assoc1 = 0.0;
    double assoc2 = 0.0;
    double cut = 0.0;
    for (int j = 0; j < num_seg1; ++j) {
      assoc1 += in_weights.row(seg1->at(j)).sum();
    }
    for (int j = 0; j < num_seg2; ++j) {
      assoc2 += in_weights.row(seg2->at(j)).sum();
    }
    for (int j = 0; j < num_seg1; ++j) {
      for (int t = 0; t < num_seg2; ++t) {
        cut += in_weights.coeffRef(seg1->at(j), seg2->at(t));
      }
    }
    float cost = static_cast<float>(cut / assoc1 + cut / assoc2);
#ifdef DEBUG_NCUT
    LOG_DEBUG << "seg1: " << num_seg1 << ", seg2: " << num_seg2 << ", split "
              << split << ", cut " << cut << ", assoc1 " << assoc1
              << ", assoc2 " << assoc2 << ", cost " << cost;
#endif
    // .2.3 find best cost
    if (cost < opt_cost) {
      opt_cost = cost;
      opt_split = split;
    }
  }
  // .3 split data according to best split
  num_seg1 = 0;
  num_seg2 = 0;
  for (int i = 0; i < num_clusters; ++i) {
    if (eigenvectors.coeffRef(i, 1) > opt_split) {
      (*seg1)[num_seg1++] = in_clusters->at(i);
    } else {
      (*seg2)[num_seg2++] = in_clusters->at(i);
    }
  }
  seg1->resize(num_seg1);
  seg2->resize(num_seg2);
  return opt_cost;
}

void NCut::LaplacianDecomposition(const Eigen::MatrixXf &weights,
                                  Eigen::MatrixXf *eigenvectors_in) {
#ifdef DEBUG_NCUT
// std::cout << "laplacian 0:\n " << weights << std::endl << std::endl;
#endif
  Eigen::MatrixXf &eigenvectors = *eigenvectors_in;
  // .1 degree matrix: D = sum(W, 2)
  Eigen::VectorXf diag(weights.rows());
  for (int i = 0; i < weights.rows(); ++i) {
    diag.coeffRef(i) = weights.row(i).sum();
  }
  // .2 graph laplacian L = D - W
  Eigen::MatrixXf laplacian(weights.rows(), weights.cols());
  for (int i = 0; i < laplacian.rows(); ++i) {
    for (int j = 0; j < laplacian.cols(); ++j) {
      if (i == j) {
        laplacian.coeffRef(i, j) = diag.coeffRef(i) - weights.coeffRef(i, j);
      } else {
        laplacian.coeffRef(i, j) = 0.f - weights.coeffRef(i, j);
      }
    }
  }
#ifdef DEBUG_NCUT
// std::cout << "laplacian 1:\n " << laplacian << std::endl << std::endl;
#endif
  // .3 D^(-1/2)
  Eigen::VectorXf diag_halfinv(weights.rows());
  for (int i = 0; i < weights.rows(); ++i) {
    diag_halfinv.coeffRef(i) =
        static_cast<float>(1.0 / std::sqrt(diag.coeffRef(i)));
  }
  // .4 normalized laplacian D^(-1/2) * L * D^(-1/2)
  for (int i = 0; i < laplacian.rows(); ++i) {
    laplacian.row(i) *= diag_halfinv.coeffRef(i);
  }
  for (int j = 0; j < laplacian.cols(); ++j) {
    laplacian.col(j) *= diag_halfinv.coeffRef(j);
  }
  // .4.2 for numerical stability, add eps to the diagonal of laplacian
  float eps = 1e-10f;
  for (int i = 0; i < laplacian.rows(); ++i) {
    laplacian.coeffRef(i, i) += eps;
  }
#ifdef DEBUG_NCUT
// std::cout << "laplacian 2:\n " << laplacian << std::endl << std::endl;
#endif
  // .5 solve eigen decompostion: TODO: lanczos
  Eigen::EigenSolver<MatrixXf> eig_solver(laplacian);
#ifdef DEBUG_NCUT
// std::cout << "eigvec 1:\n " << eig_solver.eigenvectors() << std::endl <<
// std::endl;
#endif
  // .6 sort eigen values
  std::vector<std::pair<float, int>> eigval(laplacian.rows());
  for (size_t i = 0; i < eigval.size(); ++i) {
    eigval[i] = std::make_pair(eig_solver.eigenvalues()[i].real(), i);
  }
  std::sort(eigval.begin(), eigval.end(), std::less<std::pair<float, int>>());
  // .7 get sorted eigen vectors
  eigenvectors.resize(weights.rows(), weights.cols());
  for (int i = 0; i < eigenvectors.cols(); ++i) {
    eigenvectors.col(i) =
        eig_solver.eigenvectors().col(eigval[i].second).real();
  }
  for (int i = 0; i < eigenvectors.rows(); ++i) {
    eigenvectors.row(i) *= diag_halfinv.coeffRef(i);
  }
}

bool NCut::ComputeSquaredSkeletonDistance(const Eigen::MatrixXf &in1_points,
                                          const Eigen::MatrixXf &in1_features,
                                          const Eigen::MatrixXf &in2_points,
                                          const Eigen::MatrixXf &in2_features,
                                          float *dist_point,
                                          float *dist_feature) {
  if (!((in1_points.rows() == in1_features.rows()) &&
        (in2_points.rows() == in2_features.rows()))) {
    return false;
  }
  const int num1 = static_cast<int>(in1_points.rows());
  const int num2 = static_cast<int>(in2_points.rows());
  const int dim = static_cast<int>(in1_features.cols());
  int min_index1 = -1;
  int min_index2 = -1;
  float min_dist = FLT_MAX;
  for (int i = 0; i < num1; ++i) {
    for (int j = 0; j < num2; ++j) {
      const float diff_x =
          in1_points.coeffRef(i, 0) - in2_points.coeffRef(j, 0);
      const float diff_y =
          in1_points.coeffRef(i, 1) - in2_points.coeffRef(j, 1);
      const float diff_z =
          in1_points.coeffRef(i, 2) - in2_points.coeffRef(j, 2);
      float dist = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
      if (dist < min_dist) {
        min_dist = dist;
        min_index1 = i;
        min_index2 = j;
      }
    }
  }
  *dist_point = min_dist;
  *dist_feature = 0.f;
  for (int i = 0; i < dim; ++i) {
    float diff = in1_features.coeffRef(min_index1, i) -
                 in2_features.coeffRef(min_index2, i);
    *dist_feature += diff * diff;
  }
  return true;
}

bool NCut::IsMovableObstacle(const std::vector<int> &cluster_ids,
                             std::string *label) {
  NcutBoundingBox box;
  GetComponentBoundingBox(cluster_ids, &box);
  float dummy_length = GetBboxLength(box);
  float dummy_width = GetBboxWidth(box);
  float length = std::max(dummy_length, dummy_width);
  float width = std::min(dummy_length, dummy_width);
  if (IsPotentialPedestrianSize(length, width) ||
      IsPotentialBicyclistSize(length, width) ||
      IsPotentialCarSize(length, width)) {
    *label = GetClustersLabel(cluster_ids);
    if (*label != "unknown" || *label != "background") {
      return true;
    }
  }
  return false;
}

std::string NCut::GetClustersLabel(const std::vector<int> &cluster_ids) {
  std::vector<int> point_ids;
  GetClustersPids(cluster_ids, &point_ids);
  base::PointFCloudPtr cloud =
      base::PointFCloudPtr(new base::PointFCloud(*_cloud_obstacles, point_ids));
  return GetPcLabel(cloud);
}

void NCut::GetClustersPids(const std::vector<int> &cids,
                           std::vector<int> *pids_in) {
  std::vector<int> &pids = *pids_in;
  int num_points = 0;
  for (size_t i = 0; i < cids.size(); ++i) {
    num_points += static_cast<int>(_cluster_points[cids[i]].size());
  }
  pids.resize(num_points, -1);
  int offset = 0;
  for (size_t i = 0; i < cids.size(); ++i) {
    const std::vector<int> &curr_pids = _cluster_points[cids[i]];
    memcpy(pids.data() + offset, curr_pids.data(),
           sizeof(int) * curr_pids.size());
    offset += static_cast<int>(curr_pids.size());
  }
}

int NCut::GetComponentBoundingBox(const std::vector<int> &cluster_ids,
                                  NcutBoundingBox *box_in) {
  NcutBoundingBox &box = *box_in;
  if (cluster_ids.size() == 0) {
    return 0;
  }
  int cid = cluster_ids[0];
  float x_min = std::get<0>(_cluster_bounding_box[cid]);
  float x_max = std::get<1>(_cluster_bounding_box[cid]);
  float y_min = std::get<2>(_cluster_bounding_box[cid]);
  float y_max = std::get<3>(_cluster_bounding_box[cid]);
  float z_min = std::get<4>(_cluster_bounding_box[cid]);
  float z_max = std::get<5>(_cluster_bounding_box[cid]);
  int num_points = static_cast<int>(_cluster_points[cid].size());
  for (size_t i = 1; i < cluster_ids.size(); ++i) {
    cid = cluster_ids[i];
    x_min = std::min(x_min, std::get<0>(_cluster_bounding_box[cid]));
    x_max = std::max(x_max, std::get<1>(_cluster_bounding_box[cid]));
    y_min = std::min(y_min, std::get<2>(_cluster_bounding_box[cid]));
    y_max = std::max(y_max, std::get<3>(_cluster_bounding_box[cid]));
    z_min = std::min(y_min, std::get<4>(_cluster_bounding_box[cid]));
    z_max = std::max(y_max, std::get<5>(_cluster_bounding_box[cid]));
    num_points += static_cast<int>(_cluster_points[cid].size());
  }
  std::get<0>(box) = x_min;
  std::get<1>(box) = x_max;
  std::get<2>(box) = y_min;
  std::get<3>(box) = y_max;
  std::get<4>(box) = z_min;
  std::get<5>(box) = z_max;
  return num_points;
}

std::string NCut::GetPcRoughLabel(const base::PointFCloudPtr &cloud,
                                  bool only_check_pedestrian) {
  if (cloud->size() < OBSTACLE_MINIMUM_NUM_POINTS) {
    return "unknown";
  }
  float x_max = -FLT_MAX;
  float y_max = -FLT_MAX;
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  for (size_t i = 0; i < cloud->size(); ++i) {
    base::PointF pt = (*cloud)[i];
    x_min = std::min(x_min, pt.x);
    x_max = std::max(x_max, pt.x);
    y_min = std::min(y_min, pt.y);
    y_max = std::max(y_max, pt.y);
  }
  float dummy_length = x_max - x_min;
  float dummy_width = y_max - y_min;
  float length = std::max(dummy_length, dummy_width);
  float width = std::min(dummy_length, dummy_width);
  std::string label = "unknown";
  bool is_candidate = false;
  if (only_check_pedestrian) {
    if (IsPotentialPedestrianSize(length, width) ||
        IsPotentialBicyclistSize(length, width)) {
      is_candidate = true;
    }
  } else {
    if (IsPotentialPedestrianSize(length, width) ||
        IsPotentialBicyclistSize(length, width) ||
        IsPotentialCarSize(length, width)) {
      is_candidate = true;
    }
  }
  if (is_candidate) {
    label = GetPcLabel(cloud);
  }
  return label;
}

void NCut::GetSegmentRoughSize(const base::PointFCloudPtr &cloud, float *length,
                               float *width, float *height) {
  float x_max = -FLT_MAX;
  float y_max = -FLT_MAX;
  float z_max = -FLT_MAX;
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  for (size_t i = 0; i < cloud->size(); ++i) {
    base::PointF pt = (*cloud)[i];
    x_min = std::min(x_min, pt.x);
    x_max = std::max(x_max, pt.x);
    y_min = std::min(y_min, pt.y);
    y_max = std::max(y_max, pt.y);
    z_min = std::min(z_min, pt.z);
    z_max = std::max(z_max, pt.z);
  }
  *length = (x_max - x_min);
  *width = (y_max - y_min);
  *height = (z_max - z_min);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
