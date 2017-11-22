/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#ifndef PERCEPTION_MUNKRES_H
#define PERCEPTION_MUNKRES_H

#include "opencv2/opencv.hpp"

namespace apollo {
namespace perception {
namespace traffic_light {
class Munkres {
 public:
  Munkres();

  ~Munkres() = default;;

  void Solve(cv::Mat_<int> &mat);

  void Diag(bool);

 private:
  static const int NORMAL = 0;
  static const int STAR = 1;
  static const int PRIME = 2;

  inline bool FindUncoveredInMatrix(double, unsigned int &, unsigned int &) const;

  inline bool PairInList(const std::pair<int, int> &,
                         const std::list<std::pair<int, int> > &);

  int Step1(void);

  int Step2(void);

  int Step3(void);

  int Step4(void);

  int Step5(void);

  int Step6(void);

  cv::Mat_<int> matrix_;
  cv::Mat_<int> mask_matrix_;
  bool *row_mask_;
  bool *col_mask_;
  unsigned int saverow_;
  unsigned int savecol_;
  bool is_diag_;
};
}
}
}

#endif //PERCEPTION_MUNKRES_H
