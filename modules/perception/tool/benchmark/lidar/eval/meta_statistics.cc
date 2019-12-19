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

#include "modules/perception/tool/benchmark/lidar/eval/meta_statistics.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace perception {
namespace benchmark {

bool OrientationSimilarityMetric::penalize_pi = false;

void OrientationSimilarityMetric::cal_orientation_similarity(
    const ObjectPtr& object, const ObjectPtr& gt_object) {
  if (object.get() == nullptr || gt_object.get() == nullptr) {
    return;
  }
  // yaw range is (-PI, PI]
  double gt_theta = gt_object->yaw;
  double theta = object->yaw;
  double theta_diff = fabs(gt_theta - theta);
  delta = theta_diff > M_PI ? 2 * M_PI - theta_diff : theta_diff;
  if (penalize_pi) {
    similarity = (1.0 + cos(delta)) / 2.0;
  } else {
    delta = delta > M_PI / 2 ? M_PI - delta : delta;
    similarity = (1 - sin(delta));
  }
}

std::unique_ptr<BaseRangeInterface> MetaStatistics::_s_range_interface(
    new DistanceBasedRangeInterface);
unsigned int MetaStatistics::_s_recall_dim = 41;

void MetaStatistics::set_range_type(RangeType type) {
  switch (type) {
    case VIEW:
      _s_range_interface.reset(new ViewBasedRangeInterface);
      break;
    case DISTANCE:
      _s_range_interface.reset(new DistanceBasedRangeInterface);
      break;
    case BOX:
      _s_range_interface.reset(new BoxBasedRangeInterface);
      break;
    case ROI:
      _s_range_interface.reset(new RoiDistanceBasedRangeInterface);
      break;
    default:
      _s_range_interface.reset(new DistanceBasedRangeInterface);
  }
}

void MetaStatistics::set_recall_dim(unsigned int recall_dim) {
  _s_recall_dim = recall_dim;
}

unsigned int MetaStatistics::get_type_index(const ObjectType& type) {
  unsigned index = translate_type_to_index(type);
  return index;
}

unsigned int MetaStatistics::get_range_index(const PositionMetric& position) {
  return _s_range_interface->get_index(position);
}

unsigned int MetaStatistics::get_confidence_index(double confidence) {
  unsigned int index = static_cast<unsigned int>(confidence / 0.0001);
  return index;
}

unsigned int MetaStatistics::get_type_dim() {
  return 4;  // should be sync with get_type_index and object.h!
}

unsigned int MetaStatistics::get_range_dim() {
  return _s_range_interface->get_dim();
}

unsigned int MetaStatistics::get_confidence_dim() {
  return 10001;  // should be sync with get_confidence_index!
}

std::string MetaStatistics::get_type(unsigned int index) {
  if (index >= get_type_dim()) {
    return "";
  } else {
    return translate_type_index_to_string(index);
  }
}

std::string MetaStatistics::get_range(unsigned int index) {
  return _s_range_interface->get_element(index);
}

double MetaStatistics::get_confidence(unsigned int index) {
  if (index >= get_confidence_dim()) {
    return -1.0;
  } else {
    return 0.01 * index;
  }
}

unsigned int MetaStatistics::get_recall_dim() { return _s_recall_dim; }

MetaStatistics::MetaStatistics() {}

void MetaStatistics::reset() {
  _total_detection_num.assign(get_range_dim(), 0);
  _total_groundtruth_num.assign(get_range_dim(), 0);
  _total_ji_match_num.assign(get_range_dim(), 0);
  _total_visible_groundtruth_num.assign(get_range_dim(), 0);
  _total_visible_ji_match_num.assign(get_range_dim(), 0);
  _total_hit_match_num.assign(get_range_dim(), 0);
  _total_ji_sum.assign(get_range_dim(), 0.0);
  _total_yaw_angle_diff.assign(get_range_dim(), 0.0);

  _cumulated_match_num_per_conf.assign(
      get_type_dim(), std::vector<unsigned int>(get_confidence_dim(), 0));
  _cumulated_det_alone_per_conf.assign(
      get_type_dim(), std::vector<unsigned int>(get_confidence_dim(), 0));
  _cumulated_orientation_similarity_sum_per_conf.assign(get_confidence_dim(),
                                                        0);

  _confusion_matrix.assign(
      get_range_dim(),
      std::vector<std::vector<unsigned int>>(
          get_type_dim(), std::vector<unsigned int>(get_type_dim(), 0)));

  _det_alone.assign(get_range_dim(),
                    std::vector<unsigned int>(get_type_dim(), 0));
  _gt_alone.assign(get_range_dim(),
                   std::vector<unsigned int>(get_type_dim(), 0));

  _underseg_gt_num.assign(get_type_dim(), 0);
}

template <typename T>
void vec_add1(std::vector<T>* dst, const std::vector<T>& src) {
  for (std::size_t i = 0; i < dst->size(); ++i) {
    dst->at(i) += src[i];
  }
}

template <typename T>
void vec_add2(std::vector<std::vector<T>>* dst,
              const std::vector<std::vector<T>>& src) {
  for (std::size_t i = 0; i < dst->size(); ++i) {
    for (std::size_t j = 0; j < dst->at(0).size(); ++j) {
      dst->at(i)[j] += src[i][j];
    }
  }
}

template <typename T>
void vec_add3(std::vector<std::vector<std::vector<T>>>* dst,
              const std::vector<std::vector<std::vector<T>>>& src) {
  for (std::size_t i = 0; i < dst->size(); ++i) {
    for (std::size_t j = 0; j < dst->at(0).size(); ++j) {
      for (std::size_t k = 0; k < dst->at(0)[0].size(); ++k) {
        dst->at(i)[j][k] += src[i][j][k];
      }
    }
  }
}

void compute_ap_aos(
    const std::vector<unsigned int>& cumulated_match_num_per_conf,
    const std::vector<unsigned int>& cumulated_detection_num_per_conf,
    const unsigned int total_gt_num, const unsigned int recall_dim, double* ap,
    std::vector<SPRCTuple>* tuples,
    const std::vector<double>& cumulated_orientation_similarity_per_conf,
    double* aos) {
  if (ap == nullptr || tuples == nullptr || aos == nullptr) {
    return;
  }
  *ap = 0.0;
  tuples->assign(recall_dim, SPRCTuple());

  unsigned int confidence_dim =
      static_cast<unsigned int>(cumulated_match_num_per_conf.size());

  bool with_aos = false;
  if (aos != nullptr &&
      cumulated_orientation_similarity_per_conf.size() == confidence_dim) {
    with_aos = true;
    *aos = 0.0;
  }

  if (total_gt_num == 0) {
    return;
  }
  int total_det_num = 0;
  int total_tp_num = 0;

  // calculate SPRC tuples
  double recall_step = 1.0 / (recall_dim - 1);
  double confidence_step = 1.0 / (confidence_dim - 1);
  unsigned int prc_id = 0;
  double current_recall = 0.0;
  for (int i = confidence_dim - 1; i >= 0; --i) {
    total_det_num = cumulated_detection_num_per_conf[i];
    total_tp_num = cumulated_match_num_per_conf[i];
    double left_recall = static_cast<double>(total_tp_num) / total_gt_num;
    double right_recall = 0.0;
    if (i > 0) {
      right_recall = static_cast<double>(cumulated_match_num_per_conf[i - 1]) /
                     total_gt_num;
    } else {
      right_recall = left_recall;
    }
    // if (total_gt_num == 0) {
    //     left_recall = 0.0;
    //     right_recall = 0.0;
    // }

    if ((right_recall - current_recall) < (current_recall - left_recall) &&
        i > 0) {
      continue;
    }

    if (with_aos) {
      tuples->at(prc_id).similarity =
          total_det_num > 0
              ? cumulated_orientation_similarity_per_conf[i] / total_det_num
              : 1.0;
    }

    double precision = total_det_num > 0
                           ? static_cast<double>(total_tp_num) / total_det_num
                           : 1.0;
    tuples->at(prc_id).precision = precision;
    *ap += precision;
    tuples->at(prc_id).recall = current_recall;
    tuples->at(prc_id).confidence = confidence_step * i;

    prc_id++;
    current_recall += recall_step;
    if (prc_id == recall_dim) {
      break;
    }
  }
  if (with_aos) {
    // smooth the similarity values
    *aos += tuples->at(prc_id - 1).similarity;
    for (int i = prc_id - 2; i >= 0; --i) {
      tuples->at(i).similarity =
          std::max(tuples->at(i).similarity, tuples->at(i + 1).similarity);
      *aos += tuples->at(i).similarity;
    }
    *aos /= recall_dim;
  }
  *ap /= recall_dim;

  // filling the redundant PRC tuples
  for (; prc_id < recall_dim; ++prc_id) {
    tuples->at(prc_id).recall = current_recall;
    current_recall += recall_step;
  }
}

MetaStatistics& MetaStatistics::operator+=(const MetaStatistics& rhs) {
  vec_add1(&_total_detection_num, rhs._total_detection_num);
  vec_add1(&_total_groundtruth_num, rhs._total_groundtruth_num);
  vec_add1(&_total_ji_match_num, rhs._total_ji_match_num);
  vec_add1(&_total_visible_groundtruth_num, rhs._total_visible_groundtruth_num);
  vec_add1(&_total_visible_ji_match_num, rhs._total_visible_ji_match_num);
  vec_add1(&_total_hit_match_num, rhs._total_hit_match_num);
  vec_add1(&_total_ji_sum, rhs._total_ji_sum);
  // aad and aos
  vec_add1(&_total_yaw_angle_diff, rhs._total_yaw_angle_diff);

  vec_add2(&_cumulated_match_num_per_conf, rhs._cumulated_match_num_per_conf);
  vec_add2(&_cumulated_det_alone_per_conf, rhs._cumulated_det_alone_per_conf);
  // aad and aos
  vec_add1(&_cumulated_orientation_similarity_sum_per_conf,
           rhs._cumulated_orientation_similarity_sum_per_conf);

  vec_add3(&_confusion_matrix, rhs._confusion_matrix);
  vec_add2(&_det_alone, rhs._det_alone);
  vec_add2(&_gt_alone, rhs._gt_alone);

  vec_add1(&_underseg_gt_num, rhs._underseg_gt_num);

  return *this;
}

void MetaStatistics::get_2017_detection_precision_and_recall(
    std::vector<double>* precisions, std::vector<double>* recalls) const {
  if (precisions == nullptr || recalls == nullptr) {
    return;
  }
  precisions->resize(get_range_dim() + 1, 0.0);
  recalls->resize(get_range_dim() + 1, 0.0);
  unsigned int total_detection_num_sum = 0;
  unsigned int total_groundtruth_num_sum = 0;
  unsigned int total_match_num_sum = 0;
  for (std::size_t i = 0; i < get_range_dim(); ++i) {
    precisions->at(i) = _total_detection_num[i] > 0
                            ? static_cast<double>(_total_ji_match_num[i]) /
                                  _total_detection_num[i]
                            : 0;
    recalls->at(i) = _total_groundtruth_num[i] > 0
                         ? static_cast<double>(_total_ji_match_num[i]) /
                               _total_groundtruth_num[i]
                         : 0;
    if (i != get_range_dim() - 1) {  // ignore the last range (DontCare)
                                     // if (i != get_range_dim()) {
      total_detection_num_sum += _total_detection_num[i];
      total_groundtruth_num_sum += _total_groundtruth_num[i];
      total_match_num_sum += _total_ji_match_num[i];
    }
  }
  precisions->back() =
      total_detection_num_sum > 0
          ? static_cast<double>(total_match_num_sum) / total_detection_num_sum
          : 0;
  recalls->back() =
      total_groundtruth_num_sum > 0
          ? static_cast<double>(total_match_num_sum) / total_groundtruth_num_sum
          : 0;
}

void MetaStatistics::get_2017_detection_visible_recall(
    std::vector<double>* recalls) const {
  if (recalls == nullptr) {
    return;
  }
  recalls->resize(get_range_dim() + 1, 0.0);
  unsigned int total_groundtruth_num_sum = 0;
  unsigned int total_match_num_sum = 0;
  for (std::size_t i = 0; i < get_range_dim(); ++i) {
    recalls->at(i) = _total_visible_groundtruth_num[i] > 0
                         ? static_cast<double>(_total_visible_ji_match_num[i]) /
                               _total_visible_groundtruth_num[i]
                         : 0;
    if (i != get_range_dim() - 1) {  // ignore the last range (DontCare)
      total_groundtruth_num_sum += _total_visible_groundtruth_num[i];
      total_match_num_sum += _total_visible_ji_match_num[i];
    }
  }
  recalls->back() =
      total_groundtruth_num_sum > 0
          ? static_cast<double>(total_match_num_sum) / total_groundtruth_num_sum
          : 0;
}

void MetaStatistics::get_2017_aad(std::vector<double>* aad) const {
  if (aad == nullptr) {
    return;
  }
  unsigned int total_yaw_angle_diff_sum = 0;
  unsigned int total_match_num_sum = 0;
  aad->resize(get_range_dim() + 1, 0.0);
  for (size_t i = 0; i < get_range_dim(); ++i) {
    aad->at(i) = _total_ji_match_num[i] > 0
                     ? static_cast<double>(_total_yaw_angle_diff[i]) /
                           _total_ji_match_num[i]
                     : 0;
    if (i != get_range_dim() - 1) {
      // if (i != get_range_dim()) {
      total_yaw_angle_diff_sum +=
          static_cast<unsigned int>(_total_yaw_angle_diff[i]);
      total_match_num_sum += _total_ji_match_num[i];
    }
  }
  aad->back() =
      total_match_num_sum > 0
          ? static_cast<double>(total_yaw_angle_diff_sum) / total_match_num_sum
          : 0.0;
}

void MetaStatistics::get_2016_detection_precision_and_recall(
    std::vector<double>* precisions, std::vector<double>* recalls) const {
  precisions->resize(get_range_dim(), 0.0);
  recalls->resize(get_range_dim(), 0.0);
  for (std::size_t i = 0; i < get_range_dim(); ++i) {
    precisions->at(i) =
        _total_groundtruth_num[i] > 0
            ? static_cast<double>(_total_ji_sum[i]) / _total_groundtruth_num[i]
            : 0;
    recalls->at(i) = _total_groundtruth_num[i] > 0
                         ? static_cast<double>(_total_hit_match_num[i]) /
                               _total_groundtruth_num[i]
                         : 0;
  }
}

void MetaStatistics::get_2017_detection_ap_per_type(
    std::vector<double>* ap, std::vector<std::vector<SPRCTuple>>* tuples) {
  if (ap == nullptr || tuples == nullptr) {
    return;
  }
  ap->assign(get_type_dim(), 0.0);
  tuples->assign(get_type_dim(), std::vector<SPRCTuple>());

  // for each type
  for (unsigned int t = 0; t < get_type_dim(); ++t) {
    int gt_num = 0;
    for (unsigned int r = 0; r < get_range_dim() - 1; ++r) {
      gt_num += std::accumulate(_confusion_matrix[r][t].begin(),
                                _confusion_matrix[r][t].end(), 0);
      gt_num += _gt_alone[r][t];
    }
    std::vector<unsigned int> cumulated_detection_num_per_conf =
        _cumulated_match_num_per_conf[t];
    vec_add1(&cumulated_detection_num_per_conf,
             _cumulated_det_alone_per_conf[t]);
    compute_ap_aos(_cumulated_match_num_per_conf[t],
                   cumulated_detection_num_per_conf, gt_num, _s_recall_dim,
                   &(ap->at(t)), &(tuples->at(t)));
  }
}

void MetaStatistics::get_2017_detection_ap_aos(
    double* ap, double* aos, std::vector<SPRCTuple>* tuples) const {
  if (ap == nullptr || aos == nullptr || tuples == nullptr) {
    return;
  }

  std::vector<unsigned int> cumulated_match_num_per_conf(get_confidence_dim(),
                                                         0);
  std::vector<unsigned int> cumulated_detection_num_per_conf(
      get_confidence_dim(), 0);
  for (unsigned int t = 0; t < get_type_dim(); ++t) {
    vec_add1(&cumulated_match_num_per_conf, _cumulated_match_num_per_conf[t]);
    vec_add1(&cumulated_detection_num_per_conf,
             _cumulated_match_num_per_conf[t]);
    vec_add1(&cumulated_detection_num_per_conf,
             _cumulated_det_alone_per_conf[t]);
  }
  int total_gt_num = static_cast<int>(std::accumulate(
      _total_groundtruth_num.begin(), _total_groundtruth_num.end() - 1, 0.0));
  compute_ap_aos(cumulated_match_num_per_conf, cumulated_detection_num_per_conf,
                 total_gt_num, _s_recall_dim, ap, tuples,
                 _cumulated_orientation_similarity_sum_per_conf, aos);
}

void MetaStatistics::get_2017_classification_accuracy(
    std::vector<std::vector<double>>* accuracys) const {
  if (accuracys == nullptr) {
    return;
  }
  accuracys->resize(get_type_dim(),
                    std::vector<double>(get_range_dim() + 1, 0));
  std::vector<double> numerator_sum(get_type_dim(), 0.0);
  std::vector<double> denominator_sum(get_type_dim(), 0.0);
  for (size_t range_id = 0; range_id < get_range_dim(); ++range_id) {
    for (size_t type_id = 0; type_id < get_type_dim(); ++type_id) {
      unsigned int row_sum = static_cast<unsigned int>(
          std::accumulate(_confusion_matrix[range_id][type_id].begin(),
                          _confusion_matrix[range_id][type_id].end(), 0.0));
      unsigned int col_sum = 0;
      for (size_t type_id_r = 0; type_id_r < get_type_dim(); ++type_id_r) {
        col_sum += _confusion_matrix[range_id][type_id_r][type_id];
      }
      unsigned int numerator = _confusion_matrix[range_id][type_id][type_id];
      unsigned int denominator =
          row_sum + col_sum - numerator + _det_alone[range_id][type_id];
      accuracys->at(type_id)[range_id] =
          static_cast<double>(numerator) / denominator;
      if (range_id !=
          get_range_dim() - 1) {  // ignore the last range (DontCare)
        numerator_sum[type_id] += numerator;
        denominator_sum[type_id] += denominator;
      }
    }
  }
  for (size_t type_id = 0; type_id < get_type_dim(); ++type_id) {
    accuracys->at(type_id).back() =
        static_cast<double>(numerator_sum[type_id]) / denominator_sum[type_id];
  }
}

void MetaStatistics::get_2016_classification_accuracy(
    std::vector<std::vector<double>>* accuracys) const {
  if (accuracys == nullptr) {
    return;
  }
  accuracys->resize(get_type_dim(), std::vector<double>(get_range_dim(), 0.0));

  size_t range_num = _confusion_matrix.size();
  size_t gt_type_num = _confusion_matrix[0].size();
  for (size_t range_id = 0; range_id < range_num; ++range_id) {
    for (size_t gt_type_id = 0; gt_type_id < gt_type_num; ++gt_type_id) {
      const std::vector<unsigned int>& gt_type_confusion_vector =
          _confusion_matrix[range_id][gt_type_id];
      unsigned int gt_num = std::accumulate(gt_type_confusion_vector.begin(),
                                            gt_type_confusion_vector.end(), 0);
      accuracys->at(gt_type_id)[range_id] =
          static_cast<double>(gt_type_confusion_vector[gt_type_id]) / gt_num;
    }
  }
}

void MetaStatistics::get_classification_confusion_matrix(
    std::vector<std::vector<double>>* matrix_gt_major,
    std::vector<std::vector<double>>* matrix_det_major,
    std::vector<std::vector<double>>* matrix_det_major_with_fp) const {
  if (matrix_gt_major == nullptr || matrix_det_major == nullptr) {
    return;
  }
  matrix_gt_major->resize(get_type_dim(),
                          std::vector<double>(get_type_dim(), 0.0));
  matrix_det_major->resize(get_type_dim(),
                           std::vector<double>(get_type_dim(), 0.0));
  matrix_det_major_with_fp->resize(get_type_dim(),
                                   std::vector<double>(get_type_dim(), 0.0));

  // get sum confusion matrix through different range
  size_t range_num = _confusion_matrix.size();
  size_t gt_type_num = _confusion_matrix[0].size();
  size_t det_type_num = _confusion_matrix[0][0].size();
  std::vector<std::vector<unsigned int>> sum_confusion_matrix;
  sum_confusion_matrix.resize(gt_type_num,
                              std::vector<unsigned int>(det_type_num, 0));
  for (size_t range_id = 0; range_id < range_num; ++range_id) {
    vec_add2<unsigned int>(&sum_confusion_matrix, _confusion_matrix[range_id]);
  }
  // normalize through ground truth type
  for (size_t gt_type_id = 0; gt_type_id < gt_type_num; ++gt_type_id) {
    const std::vector<unsigned int>& gt_type_confusion_vector =
        sum_confusion_matrix[gt_type_id];
    unsigned int gt_num = std::accumulate(gt_type_confusion_vector.begin(),
                                          gt_type_confusion_vector.end(), 0);
    for (size_t det_type_id = 0; det_type_id < det_type_num; ++det_type_id) {
      matrix_gt_major->at(gt_type_id)[det_type_id] =
          static_cast<double>(gt_type_confusion_vector[det_type_id]) / gt_num;
    }
  }
  // normalize through detected type
  for (size_t det_type_id = 0; det_type_id < det_type_num; ++det_type_id) {
    unsigned int det_num = 0.0;
    for (size_t gt_type_id = 0; gt_type_id < gt_type_num; ++gt_type_id) {
      det_num += sum_confusion_matrix[gt_type_id][det_type_id];
    }
    for (size_t gt_type_id = 0; gt_type_id < gt_type_num; ++gt_type_id) {
      matrix_det_major->at(det_type_id)[gt_type_id] =
          static_cast<double>(sum_confusion_matrix[gt_type_id][det_type_id]) /
          det_num;
    }
  }
  // add fp to confusion matrix, assume the corresponding type is the first
  // kind.
  for (size_t range_id = 0; range_id < get_range_dim(); ++range_id) {
    for (std::size_t type_id = 0; type_id < get_type_dim(); ++type_id) {
      sum_confusion_matrix[0][type_id] += _det_alone[range_id][type_id];
    }
  }
  for (size_t det_type_id = 0; det_type_id < det_type_num; ++det_type_id) {
    unsigned int det_num = 0.0;
    for (size_t gt_type_id = 0; gt_type_id < gt_type_num; ++gt_type_id) {
      det_num += sum_confusion_matrix[gt_type_id][det_type_id];
    }
    for (size_t gt_type_id = 0; gt_type_id < gt_type_num; ++gt_type_id) {
      matrix_det_major_with_fp->at(det_type_id)[gt_type_id] =
          static_cast<double>(sum_confusion_matrix[gt_type_id][det_type_id]) /
          det_num;
    }
  }
}

std::ostream& operator<<(std::ostream& out, const MetaStatistics& rhs) {
  out << "===========================Meta Statistics========================="
      << std::endl;
  out << "** Total detection num: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << MetaStatistics::get_range(i) << ": " << rhs._total_detection_num[i]
        << "\t";
  }
  out << std::endl;
  out << "** Total groundtruth num: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << MetaStatistics::get_range(i) << ": " << rhs._total_groundtruth_num[i]
        << "\t";
  }
  out << std::endl;
  out << "** Total ji match num: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << MetaStatistics::get_range(i) << ": " << rhs._total_ji_match_num[i]
        << "\t";
  }
  out << std::endl;
  out << "** Total visible groundtruth num: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << MetaStatistics::get_range(i) << ": "
        << rhs._total_visible_groundtruth_num[i] << "\t";
  }
  out << std::endl;
  out << "** Total visible ji match num: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << MetaStatistics::get_range(i) << ": "
        << rhs._total_visible_ji_match_num[i] << "\t";
  }
  out << std::endl;
  out << "** Total hit match num: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << MetaStatistics::get_range(i) << ": " << rhs._total_hit_match_num[i]
        << "\t";
  }
  out << std::endl;
  out << "** Total ji sum: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << MetaStatistics::get_range(i) << ": " << rhs._total_ji_sum[i] << "\t";
  }
  out << std::endl;
  // out << "cumulated match num per conf: " << std::endl;
  // for (unsigned int i = 0; i < MetaStatistics::get_confidence_dim(); ++i) {
  //    out << MetaStatistics::get_confidence(i) << ": "
  //        << rhs._cumulated_match_num_per_conf[i] <<"\t";
  //}
  // out << std::endl;
  // out << "cumulated detection num per conf: " << std::endl;
  // for (unsigned int i = 0; i < MetaStatistics::get_confidence_dim(); ++i) {
  //    out << MetaStatistics::get_confidence(i) << ": "
  //        << rhs._cumulated_detection_num_per_conf[i] <<"\t";
  //}
  // out << std::endl;
  out << "** confusion matrix(row: groundtruth, col: estimated): " << std::endl;
  for (unsigned int r = 0; r < MetaStatistics::get_range_dim(); ++r) {
    out << MetaStatistics::get_range(r) << ": " << std::endl;
    for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
      for (unsigned int j = 0; j < MetaStatistics::get_type_dim(); ++j) {
        out << MetaStatistics::get_type(i) << "--"
            << MetaStatistics::get_type(j) << ": "
            << rhs._confusion_matrix[r][i][j] << "\t";
      }
      out << std::endl;
    }
  }
  out << "** detection without groundtruth (false positive): " << std::endl;
  for (unsigned int r = 0; r < MetaStatistics::get_range_dim(); ++r) {
    out << MetaStatistics::get_range(r) << ": " << std::endl;
    for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
      out << MetaStatistics::get_type(i) << ": " << rhs._det_alone[r][i]
          << "\t";
    }
    out << std::endl;
  }
  out << "** groundtruth without detection (miss): " << std::endl;
  for (unsigned int r = 0; r < MetaStatistics::get_range_dim(); ++r) {
    out << MetaStatistics::get_range(r) << ": " << std::endl;
    for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
      out << MetaStatistics::get_type(i) << ": " << rhs._gt_alone[r][i] << "\t";
    }
    out << std::endl;
  }
  out << "** groundtruth for each class:" << std::endl;
  for (unsigned int r = 0; r < MetaStatistics::get_range_dim(); ++r) {
    out << MetaStatistics::get_range(r) << ": " << std::endl;
    for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
      unsigned int tp_num =
          std::accumulate(rhs._confusion_matrix[r][i].begin(),
                          rhs._confusion_matrix[r][i].end(), 0);
      out << MetaStatistics::get_type(i) << ": " << tp_num + rhs._gt_alone[r][i]
          << ";\t";
    }
    std::cout << std::endl;
  }
  out << "** PR for each class:" << std::endl;
  for (unsigned int r = 0; r < MetaStatistics::get_range_dim(); ++r) {
    out << MetaStatistics::get_range(r) << ": " << std::endl;
    for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
      unsigned int tp_num =
          std::accumulate(rhs._confusion_matrix[r][i].begin(),
                          rhs._confusion_matrix[r][i].end(), 0);
      out << MetaStatistics::get_type(i) << ":\trecall = "
          << static_cast<float>(tp_num) /
                 static_cast<float>((tp_num + rhs._gt_alone[r][i]))
          << ",\tprecision = "
          << static_cast<float>(tp_num) /
                 static_cast<float>((tp_num + rhs._det_alone[r][i]))
          << std::endl;
    }
  }
  out << "** under-segmented gt num: " << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
    out << MetaStatistics::get_type(i) << ":\t" << rhs._underseg_gt_num[i]
        << std::endl;
  }
  out << "==================================================================="
      << std::endl;

  return out;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
