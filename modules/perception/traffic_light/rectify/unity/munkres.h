//
// Created by gaohan02 on 17-5-23.
//

#ifndef PERCEPTION_MUNKRES_H
#define PERCEPTION_MUNKRES_H

#include "opencv2/opencv.hpp"

namespace adu {
namespace perception {
namespace traffic_light {
class Munkres {
 public:
  Munkres();

  ~Munkres() {
  };

  void solve(cv::Mat_<int> &m);

  void diag(bool);

 private:
  static const int NORMAL = 0;
  static const int STAR = 1;
  static const int PRIME = 2;

  inline bool find_uncovered_in_matrix(double, unsigned int &, unsigned int &) const;

  inline bool pair_in_list(const std::pair<int, int> &,
                           const std::list<std::pair<int, int> > &);

  int step1(void);

  int step2(void);

  int step3(void);

  int step4(void);

  int step5(void);

  int step6(void);

  cv::Mat_<int> _matrix;
  cv::Mat_<int> _mask_matrix;
  bool *_row_mask;
  bool *_col_mask;
  unsigned int _saverow;
  unsigned int _savecol;
  bool _is_diag;
};
}
}
}

#endif //PERCEPTION_MUNKRES_H
