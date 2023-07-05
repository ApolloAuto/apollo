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
#include <memory>
#include <string>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/eval/position_metric.h"
#include "modules/perception/tool/benchmark/lidar/util/object.h"

namespace apollo {
namespace perception {
namespace benchmark {

// similarity, precision, recall, confidence
struct SPRCTuple {
  double similarity = 0.0;
  double precision = 0.0;
  double recall = 0.0;
  double confidence = 0.0;
};

struct OrientationSimilarityMetric {
  void cal_orientation_similarity(const ObjectPtr& object,
                                  const ObjectPtr& gt_object);
  // if penalizes PI
  static bool penalize_pi;
  // direction angle difference
  double delta = 0.0;
  // orientation similarity
  double similarity = 0.0;
};

enum RangeType {
  DISTANCE = 0,
  VIEW = 1,
  BOX = 2,
  ROI = 3,
};

void compute_ap_aos(
    const std::vector<unsigned int>& cumulated_match_num_per_conf,
    const std::vector<unsigned int>& cumulated_detection_num_per_conf,
    const unsigned int total_gt_num, const unsigned int recall_dim, double* ap,
    std::vector<SPRCTuple>* tuples,
    const std::vector<double>& cumulated_orientation_similarity_per_conf =
        std::vector<double>(),
    double* aos = nullptr);

class MetaStatistics {
 public:
  MetaStatistics();
  ~MetaStatistics() = default;
  void reset();
  // this operator designed for online update
  MetaStatistics& operator+=(const MetaStatistics& rhs);
  // get precision and recall for each range
  void get_2017_detection_precision_and_recall(
      std::vector<double>* precisions, std::vector<double>* recalls) const;
  void get_2017_detection_visible_recall(std::vector<double>* recalls) const;
  // get 2017 orientation aad
  void get_2017_aad(std::vector<double>* aad) const;

  void get_2016_detection_precision_and_recall(
      std::vector<double>* precisions, std::vector<double>* recalls) const;
  // get ap, and pr-curve samples <precision, recall, confidence>
  void get_2017_detection_ap_aos(double* ap, double* aos,
                                 std::vector<SPRCTuple>* tuples) const;
  void get_2017_detection_ap_per_type(
      std::vector<double>* ap, std::vector<std::vector<SPRCTuple>>* tuples);
  // get 2017 classification accuracy
  void get_2017_classification_accuracy(
      std::vector<std::vector<double>>* accuracys) const;
  // get 2016 classification accuracy
  // D1: type D2: range
  void get_2016_classification_accuracy(
      std::vector<std::vector<double>>* accuracys) const;
  // get classification confusion matrix (normalized)
  void get_classification_confusion_matrix(
      std::vector<std::vector<double>>* matrix_gt_major,
      std::vector<std::vector<double>>* matrix_det_major,
      std::vector<std::vector<double>>* matrix_det_major_with_fp) const;

  friend std::ostream& operator<<(std::ostream& out, const MetaStatistics& rhs);
  // class FrameStatistics will fill in the raw data
  friend class FrameStatistics;

 public:
  static void set_range_type(RangeType type);
  static void set_recall_dim(unsigned int prc_dim);
  static unsigned int get_type_index(const ObjectType& type);
  static unsigned int get_range_index(const PositionMetric& position);
  static unsigned int get_confidence_index(double confidence);
  static unsigned int get_type_dim();
  static unsigned int get_range_dim();
  static unsigned int get_confidence_dim();
  static std::string get_type(unsigned int index);
  static std::string get_range(unsigned int index);
  static double get_confidence(unsigned int index);
  static unsigned int get_recall_dim();

 private:
  static std::unique_ptr<BaseRangeInterface> _s_range_interface;
  static unsigned int _s_recall_dim;
  // record for detection precision and recall
  // D1: range
  std::vector<unsigned int> _total_detection_num;
  std::vector<unsigned int> _total_groundtruth_num;
  // match statisfy ji > TH criteria (17 KPI)
  std::vector<unsigned int> _total_ji_match_num;
  // visible groundtruth number and matches
  std::vector<unsigned int> _total_visible_groundtruth_num;
  std::vector<unsigned int> _total_visible_ji_match_num;
  // angle difference sum in different range
  std::vector<double> _total_yaw_angle_diff;
  // match statisfy sum{N(o_i \cap o_gt)} / N(o_gt) > TH criteria (16 KPI)
  // thus allow many to one match
  std::vector<unsigned int> _total_hit_match_num;
  std::vector<double> _total_ji_sum;  // (16 KPI)
  // record for detection ap
  // D1: gt type D2: confidence (store #samples of confidence >= bin confidence)
  std::vector<std::vector<unsigned int>> _cumulated_match_num_per_conf;
  // D1: det type D2: confidence (store #samples of confidence >= bin
  // confidence)
  std::vector<std::vector<unsigned int>> _cumulated_det_alone_per_conf;
  // record for detection aos
  // D1: confidence (store #samples of confidence >= bin confidence)
  std::vector<double> _cumulated_orientation_similarity_sum_per_conf;
  // record for classification accuracy
  // D1: range D2: groundtruth type D3: estimated type
  std::vector<std::vector<std::vector<unsigned int>>> _confusion_matrix;
  // record # of types of detections without groundtruth match
  // D1: range D2: type
  std::vector<std::vector<unsigned int>> _det_alone;
  // record # of types of groundtruth without detection match
  // D1: range D2: type
  std::vector<std::vector<unsigned int>> _gt_alone;
  // record # of under-segmented gt num
  // D1: type
  std::vector<unsigned int> _underseg_gt_num;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
