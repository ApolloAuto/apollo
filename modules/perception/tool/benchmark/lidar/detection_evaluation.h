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
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/eval/frame_statistics.h"
#include "modules/perception/tool/benchmark/lidar/eval/lidar_option.h"
#include "modules/perception/tool/benchmark/lidar/eval/sequence_self_statistics.h"
#include "modules/perception/tool/benchmark/lidar/loader/async_sequence_data_loader.h"

namespace apollo {
namespace perception {
namespace benchmark {

struct FrameMetrics {
  std::string frame_name;
  std::vector<double> detection_recall_2017;
  std::vector<double> detection_precision_2017;
  std::vector<double> detection_visible_recall_2017;
  std::vector<double> aad_2017;
  double jaccard_index_percentile = 0.0;

  bool operator<(const FrameMetrics& rhs) const {
    double score = std::accumulate(detection_recall_2017.begin(),
                                   detection_recall_2017.end(), 0.0);
    double score_right = std::accumulate(rhs.detection_recall_2017.begin(),
                                         rhs.detection_recall_2017.end(), 0.0);
    return score < score_right;
  }
};

class DetectionEvaluation {
 public:
  DetectionEvaluation() = default;
  ~DetectionEvaluation() = default;
  DetectionEvaluation(const DetectionEvaluation& rhs) = delete;

  bool init(const std::string& clouds, const std::string& results,
            const std::string& groundtruths, bool is_folder,
            unsigned int loader_thread_num, unsigned int eval_thread_num,
            unsigned int eval_parrallel_num, const std::string& reserve);
  void run_evaluation();

  friend std::ostream& operator<<(std::ostream& out,
                                  const DetectionEvaluation& rhs);

 private:
  bool _initialized = false;
  std::unique_ptr<ctpl::thread_pool> _thread_pool;
  AsyncSequenceDataLoader<FrameStatistics> _loader;

  MetaStatistics _meta_stat;
  unsigned int _eval_parrallel_num = 0;

  SequenceSelfStatistics<unsigned int> _self_stat;

 private:
  // detection evaluation metrics
  // per frame
  std::vector<FrameMetrics> _frame_metrics;
  // OrientationSimilarityMetric _similarity_metric;
  // whole dataset
  std::vector<double> _detection_precision_2017;
  std::vector<double> _detection_recall_2017;
  std::vector<double> _detection_visible_recall_2017;
  std::vector<double> _detection_precision_2016;
  std::vector<double> _detection_recall_2016;
  double _detection_ap;
  double _detection_aos;
  std::vector<SPRCTuple> _detection_curve_samples;
  std::vector<double> _detection_ap_per_type;
  std::vector<std::vector<SPRCTuple>> _detection_curve_samples_per_type;
  // orientation
  std::vector<double> _aad_2017;

  std::vector<std::vector<double>> _classification_accuracy_2016;
  std::vector<std::vector<double>> _classification_accuracy_2017;
  std::vector<std::vector<double>> _classification_confusion_matrix_gt_major;
  std::vector<std::vector<double>> _classification_confusion_matrix_det_major;
  std::vector<std::vector<double>>
      _classification_confusion_matrix_det_major_with_fp;
  // self-evaluation
  std::vector<std::vector<double>> _classification_change_rate_per_class;
  double _classification_change_rate;
  // option
  LidarOption _lidar_option;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
