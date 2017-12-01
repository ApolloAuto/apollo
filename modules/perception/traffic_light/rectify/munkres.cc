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
#include "munkres.h"

namespace apollo {
namespace perception {
namespace traffic_light {
///////////////////////////
Munkres::Munkres() {
  is_diag_ = false;
}

///////////////////////////
void Munkres::Diag(bool status) {
  is_diag_ = status;
}

int MaxValue(cv::Mat_<int> &m) {
  int max = 0;  // std::numeric_limits<int>::min();
  for (int i = 0; i < m.rows; i++) {
    for (int j = 0; j < m.cols; j++) {
      max = std::max<int>(max, m(i, j));
    }
  }
  return max;
}

void ExtendMat(cv::Mat_<int> &mat, unsigned int rows, unsigned int cols,
               int value = 0) {
  cv::Size2i inter_size;
  inter_size.height = std::min<int>(rows, mat.rows);
  inter_size.width = std::min<int>(cols, mat.cols);

  cv::Mat tm(rows, cols, mat.type());
  tm.setTo(cv::Scalar(value));

  if (inter_size.width != 0 && inter_size.height != 0) {
    mat(cv::Rect(cv::Point(0, 0), inter_size))
        .copyTo(tm(cv::Rect(cv::Point(0, 0), inter_size)));
  }
  mat = tm;
}

void ReplaceInfinites(cv::Mat_<int> &mat) {
  const unsigned int rows = mat.rows;
  const unsigned int cols = mat.cols;
  //  assert( rows > 0 && columns > 0 );
  if (rows == 0 || cols == 0) {
    return;
  }
  double max = mat(0, 0);
  const double eps = 1e-6;

  const auto infinity = std::numeric_limits<int>::infinity();
  //
  // Find the greatest value in the _matrix that isn't infinity.
  for (unsigned int row = 0; row < rows; row++) {
    for (unsigned int col = 0; col < cols; col++) {
      if (mat(row, col) != infinity) {
        if (abs(max - infinity) < eps) {
          max = mat(row, col);
        } else {
          max = std::max<int>(max, mat(row, col));
        }
      }
    }
  }

  // a value higher than the maximum value present in the _matrix.
  if (abs(max - infinity) < eps) {
    // This case only occurs when all values are infinite.
    max = 0;
  } else {
    max++;
  }

  for (unsigned int row = 0; row < rows; row++) {
    for (unsigned int col = 0; col < cols; col++) {
      if (mat(row, col) == infinity) {
        mat(row, col) = max;
      }
    }
  }
}

void MinimizeAlongDirection(cv::Mat_<int> &matrix, bool over_columns) {
  const unsigned int outer_size = over_columns ? matrix.cols : matrix.rows,
                     inner_size = over_columns ? matrix.rows : matrix.cols;

  // Look for a minimum value to subtract from all values along
  // the "outer" direction.
  for (unsigned int i = 0; i < outer_size; i++) {
    double min = over_columns ? matrix(0, i) : matrix(i, 0);

    // As long as the current minimum is greater than zero,
    // keep looking for the minimum.
    // Start at one because we already have the 0th value in min.
    for (unsigned int j = 1; j < inner_size && min > 0; j++) {
      min = std::min<int>(min, over_columns ? matrix(j, i) : matrix(i, j));
    }

    if (min > 0) {
      for (unsigned int j = 0; j < inner_size; j++) {
        if (over_columns) {
          matrix(j, i) -= min;
        } else {
          matrix(i, j) -= min;
        }
      }
    }
  }
}

void Munkres::Solve(cv::Mat_<int> &mat) {
  const unsigned int rows = mat.rows;
  const unsigned int cols = mat.cols, size = std::max<unsigned int>(rows, cols);
  bool notdone = true;
  int step = 1;

  // Copy input _matrix
  this->matrix_ = mat;

  // If the input _matrix isn't square, make it square and fill the empty values
  // with the largest value present in the _matrix.
  if (rows != cols) {
    ExtendMat(matrix_, size, size, MaxValue(matrix_));
  }

  // STAR == 1 == starred, PRIME == 2 == primed
  // _mask_matrix.resize(size, size);
  ExtendMat(mask_matrix_, size, size);

  row_mask_ = new bool[size];
  col_mask_ = new bool[size];  //?columns
  for (unsigned int i = 0; i < size; i++) {
    row_mask_[i] = false;
  }
  for (unsigned int i = 0; i < size; i++) {
    col_mask_[i] = false;
  }

  // Prepare the _matrix values...  If there were any infinities, replace them
  // with a value greater than the maximum value in the _matrix.
  ReplaceInfinites(matrix_);

  MinimizeAlongDirection(matrix_, false);
  MinimizeAlongDirection(matrix_, true);

  // Follow the steps
  while (notdone) {
    switch (step) {
      case 0:
        notdone = false;
        // end the step flow
        break;
      case 1:
        step = Step1();
        // step is always 2
        break;
      case 2:
        step = Step2();
        // step is always either 0 or 3
        break;
      case 3:
        step = Step3();
        // step in [3, 4, 5]
        break;
      case 4:
        step = Step4();
        // step is always 2
        break;
      case 5:
        step = Step5();
        // step is always 3
        break;
    }
  }
  // Store results
  for (unsigned int row = 0; row < size; row++) {
    for (unsigned int col = 0; col < size; col++) {
      if (mask_matrix_(row, col) == STAR) {
        matrix_(row, col) = 0;
      } else {
        matrix_(row, col) = -1;
      }
    }
  }

  // Remove the excess rows or columns that we added to fit the
  // input to a square _matrix.
  //    _matrix.resize(rows, columns);
  ExtendMat(matrix_, rows, cols);

  mat = matrix_;

  delete[] row_mask_;
  delete[] col_mask_;
}

bool Munkres::FindUncoveredInMatrix(double item, unsigned int &row,
                                    unsigned int &col) const {
  unsigned int rows = matrix_.rows;
  unsigned int columns = matrix_.cols;

  for (row = 0; row < rows; row++) {
    if (!row_mask_[row]) {
      for (col = 0; col < columns; col++) {
        if (!col_mask_[col]) {
          if (matrix_(row, col) == item) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool Munkres::PairInList(const std::pair<int, int> &needle,
                         const std::list<std::pair<int, int>> &haystack) {
  for (std::list<std::pair<int, int>>::const_iterator i = haystack.begin();
       i != haystack.end(); i++) {
    if (needle == *i) {
      return true;
    }
  }
  return false;
}

int Munkres::Step1(void) {
  const unsigned int rows = matrix_.rows;
  const unsigned int cols = matrix_.cols;

  for (unsigned int row = 0; row < rows; row++) {
    for (unsigned int col = 0; col < cols; col++) {
      if (0 == matrix_(row, col)) {
        bool isstarred = false;
        for (unsigned int nrow = 0; nrow < rows; nrow++) {
          if (STAR == mask_matrix_(nrow, col)) {
            isstarred = true;
            break;
          }
        }

        if (!isstarred) {
          for (unsigned int ncol = 0; ncol < cols; ncol++) {
            if (STAR == mask_matrix_(row, ncol)) {
              isstarred = true;
              break;
            }
          }
        }

        if (!isstarred) {
          mask_matrix_(row, col) = STAR;
        }
      }
    }
  }
  return 2;
}

unsigned int minsize(cv::Mat_<int> &m) {
  return (m.rows < m.cols) ? m.rows : m.cols;
}
int Munkres::Step2(void) {
  const unsigned int rows = matrix_.rows, columns = matrix_.cols;
  unsigned int covercount = 0;

  for (unsigned int row = 0; row < rows; row++) {
    for (unsigned int col = 0; col < columns; col++) {
      if (STAR == mask_matrix_(row, col)) {
        col_mask_[col] = true;
        covercount++;
      }
    }
  }

  if (covercount >= minsize(matrix_)) {
    if (is_diag_) {
      std::cout << "Final cover count: " << covercount << std::endl;
    }
    return 0;
  }
  return 3;
}

int Munkres::Step3(void) {
  if (FindUncoveredInMatrix(0, saverow_, savecol_)) {
    mask_matrix_(saverow_, savecol_) = PRIME;  // prime it.
  } else {
    return 5;
  }

  for (unsigned int ncol = 0; ncol < matrix_.cols; ncol++) {
    if (mask_matrix_(saverow_, ncol) == STAR) {
      row_mask_[saverow_] = true;
      col_mask_[ncol] = false;
      return 3;
    }
  }
  return 4;
}

int Munkres::Step4(void) {
  const unsigned int rows = matrix_.rows;
  const unsigned int cols = matrix_.cols;

  // seq contains pairs of row/column values where we have found
  // either a star or a prime that is part of the ``alternating sequence``.
  std::list<std::pair<int, int>> seq;
  // use _saverow, _savecol from step 3.
  std::pair<int, int> z0(saverow_, savecol_);
  seq.insert(seq.end(), z0);

  // We have to find these two pairs:
  std::pair<int, int> z1(-1, -1);
  std::pair<int, int> z2n(-1, -1);

  unsigned int row = 0;
  unsigned int col = savecol_;
  /*
   Increment Set of Starred Zeros

   1. Construct the ``alternating sequence'' of primed and starred zeros:

   Z0 : Unpaired Z' from Step 4.2
   Z1 : The Z* in the column of Z0
   Z[2N] : The Z' in the row of Z[2N-1], if such a zero exists
   Z[2N+1] : The Z* in the column of Z[2N]

   The sequence eventually terminates with an unpaired Z' = Z[2N] for some N.
   */
  bool madepair = false;
  do {
    madepair = false;
    for (row = 0; row < rows; row++) {
      if (mask_matrix_(row, col) == STAR) {
        z1.first = row;
        z1.second = col;
        if (PairInList(z1, seq)) {
          continue;
        }
        madepair = true;
        seq.insert(seq.end(), z1);
        break;
      }
    }

    if (!madepair) {
      break;
    }

    madepair = false;

    for (col = 0; col < cols; col++) {
      if (mask_matrix_(row, col) == PRIME) {
        z2n.first = row;
        z2n.second = col;
        if (PairInList(z2n, seq)) {
          continue;
        }
        madepair = true;
        seq.insert(seq.end(), z2n);
        break;
      }
    }
  } while (madepair);

  for (std::list<std::pair<int, int>>::iterator i = seq.begin(); i != seq.end();
       i++) {
    // 2. Unstar each starred zero of the sequence.
    if (mask_matrix_(i->first, i->second) == STAR) {
      mask_matrix_(i->first, i->second) = NORMAL;
    }

    // 3. Star each primed zero of the sequence,
    // thus increasing the number of starred zeros by one.
    if (mask_matrix_(i->first, i->second) == PRIME) {
      mask_matrix_(i->first, i->second) = STAR;
    }
  }

  // 4. Erase all primes, uncover all columns and rows,
  for (unsigned int row = 0; row < mask_matrix_.rows; row++) {
    for (unsigned int col = 0; col < mask_matrix_.cols; col++) {
      if (mask_matrix_(row, col) == PRIME) {
        mask_matrix_(row, col) = NORMAL;
      }
    }
  }

  for (unsigned int i = 0; i < rows; i++) {
    row_mask_[i] = false;
  }

  for (unsigned int i = 0; i < cols; i++) {
    col_mask_[i] = false;
  }
  // and return to Step 2.
  return 2;
}

int Munkres::Step5(void) {
  const unsigned int rows = matrix_.rows, columns = matrix_.cols;
  /*
   New Zero Manufactures

   1. Let h be the smallest uncovered entry in the (modified) distance _matrix.
   2. Add h to all covered rows.
   3. Subtract h from all uncovered columns
   4. Return to Step 3, without altering stars, primes, or covers.
   */
  double h = 0;
  for (unsigned int row = 0; row < rows; row++) {
    if (!row_mask_[row]) {
      for (unsigned int col = 0; col < columns; col++) {
        if (!col_mask_[col]) {
          if ((h > matrix_(row, col) && matrix_(row, col) != 0) || h == 0) {
            h = matrix_(row, col);
          }
        }
      }
    }
  }

  for (unsigned int row = 0; row < rows; row++) {
    if (row_mask_[row]) {
      for (unsigned int col = 0; col < columns; col++) {
        matrix_(row, col) += h;
      }
    }
  }

  for (unsigned int col = 0; col < columns; col++) {
    if (!col_mask_[col]) {
      for (unsigned int row = 0; row < rows; row++) {
        matrix_(row, col) -= h;
      }
    }
  }
  return 3;
}
}
}
}