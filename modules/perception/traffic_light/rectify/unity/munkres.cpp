//
// Created by gaohan02 on 17-5-23.
//

#include "munkres.h"

namespace adu {
namespace perception {
namespace traffic_light {
///////////////////////////
Munkres::Munkres() {
  _is_diag = false;
}

///////////////////////////
void Munkres::diag(bool status) {
  _is_diag = status;
}

///////////////////////////
int max_value(cv::Mat_<int> &m) {
  int max = 0;//std::numeric_limits<int>::min();
  for (int i = 0; i < m.rows; i++) {
    for (int j = 0; j < m.cols; j++) {
      max = std::max<int>(max, m(i, j));
    }
  }
  return max;
}

///////////////////////////
void extend_mat(cv::Mat_<int> &m, unsigned int rows, unsigned int cols, int value = 0) {
  cv::Size2i inter_size;
  inter_size.height = std::min<int>(rows, m.rows);
  inter_size.width = std::min<int>(cols, m.cols);

  cv::Mat tm(rows, cols, m.type());
  tm.setTo(cv::Scalar(value));

  if (inter_size.width != 0 && inter_size.height != 0) {
    m(cv::Rect(cv::Point(0, 0), inter_size)).copyTo(
        tm(cv::Rect(cv::Point(0, 0), inter_size)));
  }
  m = tm;
}

///////////////////////////
void replace_infinites(cv::Mat_<int> &matrix) {
  const unsigned int rows = matrix.rows, columns = matrix.cols;
  //  assert( rows > 0 && columns > 0 );
  if (rows == 0 || columns == 0) {
    return;
  }
  double max = matrix(0, 0);
  const double eps = 1e-6;

  const auto infinity = std::numeric_limits<int>::infinity();
  //
  // Find the greatest value in the _matrix that isn't infinity.
  for (unsigned int row = 0; row < rows; row++) {
    for (unsigned int col = 0; col < columns; col++) {
      if (matrix(row, col) != infinity) {
        if (abs(max - infinity) < eps) {
          max = matrix(row, col);
        } else {
          max = std::max<int>(max, matrix(row, col));
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
    for (unsigned int col = 0; col < columns; col++) {
      if (matrix(row, col) == infinity) {
        matrix(row, col) = max;
      }
    }
  }

}

///////////////////////////
void minimize_along_direction(cv::Mat_<int> &matrix, bool over_columns) {
  const unsigned int outer_size = over_columns ? matrix.cols
                                               : matrix.rows, inner_size = over_columns
                                                                           ? matrix.rows
                                                                           : matrix.cols;

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

///////////////////////////
void Munkres::solve(cv::Mat_<int> &m) {
  const unsigned int rows = m.rows, columns = m.cols, size = std::max<unsigned int>(rows,
                                                                                    columns);
  bool notdone = true;
  int step = 1;

  // Copy input _matrix
  this->_matrix = m;

  // If the input _matrix isn't square, make it square and fill the empty values with the largest value present in the _matrix.
  if (rows != columns) {
    extend_mat(_matrix, size, size, max_value(_matrix));
  }

  // STAR == 1 == starred, PRIME == 2 == primed
  // _mask_matrix.resize(size, size);
  extend_mat(_mask_matrix, size, size);

  _row_mask = new bool[size];
  _col_mask = new bool[size];//?columns
  for (unsigned int i = 0; i < size; i++) {
    _row_mask[i] = false;
  }
  for (unsigned int i = 0; i < size; i++) {
    _col_mask[i] = false;
  }

  // Prepare the _matrix values...  If there were any infinities, replace them with a value greater than the maximum value in the _matrix.
  replace_infinites(_matrix);

  minimize_along_direction(_matrix, false);
  minimize_along_direction(_matrix, true);

  // Follow the steps
  while (notdone) {
    switch (step) {
      case 0:notdone = false;
        // end the step flow
        break;
      case 1:step = step1();
        // step is always 2
        break;
      case 2:step = step2();
        // step is always either 0 or 3
        break;
      case 3:step = step3();
        // step in [3, 4, 5]
        break;
      case 4:step = step4();
        // step is always 2
        break;
      case 5:step = step5();
        // step is always 3
        break;
    }
  }
  // Store results
  for (unsigned int row = 0; row < size; row++) {
    for (unsigned int col = 0; col < size; col++) {
      if (_mask_matrix(row, col) == STAR) {
        _matrix(row, col) = 0;
      } else {
        _matrix(row, col) = -1;
      }
    }
  }

  // Remove the excess rows or columns that we added to fit the
  // input to a square _matrix.
  //    _matrix.resize(rows, columns);
  extend_mat(_matrix, rows, columns);

  m = _matrix;

  delete[] _row_mask;
  delete[] _col_mask;

}

///////////////////////////
bool Munkres::find_uncovered_in_matrix(double item, unsigned int &row, unsigned int &col) const {
  unsigned int rows = _matrix.rows;
  unsigned int columns = _matrix.cols;

  for (row = 0; row < rows; row++) {
    if (!_row_mask[row]) {
      for (col = 0; col < columns; col++) {
        if (!_col_mask[col]) {
          if (_matrix(row, col) == item) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

///////////////////////////
bool Munkres::pair_in_list(const std::pair<int, int> &needle,
                           const std::list<std::pair<int, int> > &haystack) {
  for (std::list<std::pair<int, int> >::const_iterator i = haystack.begin();
      i != haystack.end(); i++) {
    if (needle == *i) {
      return true;
    }
  }
  return false;
}

///////////////////////////
int Munkres::step1(void) {
  const unsigned int rows = _matrix.rows;
  const unsigned int columns = _matrix.cols;

  for (unsigned int row = 0; row < rows; row++) {
    for (unsigned int col = 0; col < columns; col++) {
      if (0 == _matrix(row, col)) {
        bool isstarred = false;
        for (unsigned int nrow = 0; nrow < rows; nrow++) {
          if (STAR == _mask_matrix(nrow, col)) {
            isstarred = true;
            break;
          }
        }

        if (!isstarred) {
          for (unsigned int ncol = 0; ncol < columns; ncol++) {
            if (STAR == _mask_matrix(row, ncol)) {
              isstarred = true;
              break;
            }
          }
        }

        if (!isstarred) {
          _mask_matrix(row, col) = STAR;
        }
      }
    }
  }
  return 2;
}

///////////////////////////
unsigned int minsize(cv::Mat_<int> &m) {
  return (m.rows < m.cols) ? m.rows : m.cols;
}
int Munkres::step2(void) {
  const unsigned int rows = _matrix.rows, columns = _matrix.cols;
  unsigned int covercount = 0;

  for (unsigned int row = 0; row < rows; row++) {
    for (unsigned int col = 0; col < columns; col++) {
      if (STAR == _mask_matrix(row, col)) {
        _col_mask[col] = true;
        covercount++;
      }
    }
  }

  if (covercount >= minsize(_matrix)) {
    if (_is_diag) {
      std::cout << "Final cover count: " << covercount << std::endl;
    }
    return 0;
  }
  return 3;
}

///////////////////////////
int Munkres::step3(void) {
  /*
   Main Zero Search
   1. Find an uncovered Z in the distance _matrix and prime it. If no such zero exists, go to Step 5
   2. If No Z* exists in the row of the Z', go to Step 4.
   3. If a Z* exists, cover this row and uncover the column of the Z*. Return to Step 3.1 to find a new Z
   */
  if (find_uncovered_in_matrix(0, _saverow, _savecol)) {
    _mask_matrix(_saverow, _savecol) = PRIME; // prime it.
  } else {
    return 5;
  }

  for (unsigned int ncol = 0; ncol < _matrix.cols; ncol++) {
    if (_mask_matrix(_saverow, ncol) == STAR) {
      _row_mask[_saverow] = true; //cover this row and
      _col_mask[ncol] = false; // uncover the column containing the starred zero
      return 3; // repeat
    }
  }
  return 4; // no starred zero in the row containing this primed zero
}

///////////////////////////
int Munkres::step4(void) {
  const unsigned int rows = _matrix.rows,
      columns = _matrix.cols;

  // seq contains pairs of row/column values where we have found
  // either a star or a prime that is part of the ``alternating sequence``.
  std::list<std::pair<int, int> > seq;
  // use _saverow, _savecol from step 3.
  std::pair<int, int> z0(_saverow, _savecol);
  seq.insert(seq.end(), z0);

  // We have to find these two pairs:
  std::pair<int, int> z1(-1, -1);
  std::pair<int, int> z2n(-1, -1);

  unsigned int row = 0;
  unsigned int col = _savecol;
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
      if (_mask_matrix(row, col) == STAR) {
        z1.first = row;
        z1.second = col;
        if (pair_in_list(z1, seq)) {
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

    for (col = 0; col < columns; col++) {
      if (_mask_matrix(row, col) == PRIME) {
        z2n.first = row;
        z2n.second = col;
        if (pair_in_list(z2n, seq)) {
          continue;
        }
        madepair = true;
        seq.insert(seq.end(), z2n);
        break;
      }
    }
  } while (madepair);

  for (std::list<std::pair<int, int> >::iterator i = seq.begin(); i != seq.end(); i++) {
    // 2. Unstar each starred zero of the sequence.
    if (_mask_matrix(i->first, i->second) == STAR) {
      _mask_matrix(i->first, i->second) = NORMAL;
    }

    // 3. Star each primed zero of the sequence,
    // thus increasing the number of starred zeros by one.
    if (_mask_matrix(i->first, i->second) == PRIME) {
      _mask_matrix(i->first, i->second) = STAR;
    }
  }

  // 4. Erase all primes, uncover all columns and rows,
  for (unsigned int row = 0; row < _mask_matrix.rows; row++) {
    for (unsigned int col = 0; col < _mask_matrix.cols; col++) {
      if (_mask_matrix(row, col) == PRIME) {
        _mask_matrix(row, col) = NORMAL;
      }
    }
  }

  for (unsigned int i = 0; i < rows; i++) {
    _row_mask[i] = false;
  }

  for (unsigned int i = 0; i < columns; i++) {
    _col_mask[i] = false;
  }
  // and return to Step 2.
  return 2;
}

///////////////////////////
int Munkres::step5(void) {
  const unsigned int rows = _matrix.rows, columns = _matrix.cols;
  /*
   New Zero Manufactures

   1. Let h be the smallest uncovered entry in the (modified) distance _matrix.
   2. Add h to all covered rows.
   3. Subtract h from all uncovered columns
   4. Return to Step 3, without altering stars, primes, or covers.
   */
  double h = 0;
  for (unsigned int row = 0; row < rows; row++) {
    if (!_row_mask[row]) {
      for (unsigned int col = 0; col < columns; col++) {
        if (!_col_mask[col]) {
          if ((h > _matrix(row, col) && _matrix(row, col) != 0) || h == 0) {
            h = _matrix(row, col);
          }
        }
      }
    }
  }

  for (unsigned int row = 0; row < rows; row++) {
    if (_row_mask[row]) {
      for (unsigned int col = 0; col < columns; col++) {
        _matrix(row, col) += h;
      }
    }
  }

  for (unsigned int col = 0; col < columns; col++) {
    if (!_col_mask[col]) {
      for (unsigned int row = 0; row < rows; row++) {
        _matrix(row, col) -= h;
      }
    }
  }
  return 3;
}

}
}
}