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

#include <math.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include <google/protobuf/text_format.h>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace apollo {
namespace dreamview {

class SimControlUtil {
 public:
  double interpolate_1d(const double& p1, const double& p2,
                        const double& frac1);
  double interpolated_find(const std::vector<double>& range_table,
                           const std::vector<double>& val_table,
                           double to_find);

  static double sigmoid(const double value);
  static double relu(const double value);
  double normalize(const double value, const double mean, const double std);

  template <class P>
  bool load_binary_file(const std::string& filename, P* pb_out) {
    std::fstream input(filename, std::ios::in | std::ios::binary);
    return pb_out->ParseFromIstream(&input);
  }

  template <class P>
  bool load_text_file(const std::string& filename, P* pb_out) {
    std::fstream input(filename, std::ios::in);
    std::string input_data((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());

    if (input_data.empty()) {
      return false;
    }

    return google::protobuf::TextFormat::ParseFromString(input_data, pb_out);
  }

  template <class P>
  bool load_file_to_proto(const std::string& filename, P* pb_out) {
    if (ends_with(filename, ".bin")) {
      if (!load_binary_file(filename, pb_out) &&
          !load_text_file(filename, pb_out)) {
        return false;
      }
    } else {
      if (!load_text_file(filename, pb_out) &&
          !load_binary_file(filename, pb_out)) {
        return false;
      }
    }

    return true;
  }

  bool ends_with(const std::string& original, const std::string& pattern) {
    return original.length() >= pattern.length() &&
           original.substr(original.length() - pattern.length()) == pattern;
  }

  // delta function
  int delta_function(double value, double threshold) {
    return static_cast<int>(value > threshold);
  }

  // calculate \eta from rise time and peak time
  double get_eta(double rise_time, double peak_time) {
    if (peak_time <= 0.0) {
      AFATAL << "peak_time should be positive";
    }
    return cos(M_PI - rise_time / (peak_time / M_PI));
  }

  // calculate \tau_s (1/omega); only suitable for under-damped system (eta < 1)
  double get_tau_s(double peak_time, double eta) {
    if (eta > 1.0) {
      AFATAL << "not an underdamped system";
    }
    return peak_time / sqrt(1 - eta * eta);
  }
};

}  // namespace dreamview
}  // namespace apollo
