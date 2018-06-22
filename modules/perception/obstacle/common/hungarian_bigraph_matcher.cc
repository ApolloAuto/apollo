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

#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"

#include <iostream>

namespace apollo {
namespace perception {

HungarianOptimizer::HungarianOptimizer(
    const std::vector<std::vector<double>>& costs)
    : state_(nullptr) {
  width_ = costs.size();

  if (width_ > 0) {
    height_ = costs[0].size();
  } else {
    height_ = 0;
  }

  matrix_size_ = std::max(width_, height_);
  max_cost_ = 0;

  // Generate the expanded cost matrix by adding extra 0-valued elements in
  // order to make a square matrix.  At the same time, find the greatest cost
  // in the matrix (used later if we want to maximize rather than minimize the
  // overall cost.)
  costs_.resize(matrix_size_);
  for (int row = 0; row < matrix_size_; ++row) {
    costs_[row].resize(matrix_size_);
  }

  for (int row = 0; row < matrix_size_; ++row) {
    for (int col = 0; col < matrix_size_; ++col) {
      if ((row >= width_) || (col >= height_)) {
        costs_[row][col] = 0;
      } else {
        costs_[row][col] = costs[row][col];
        max_cost_ = std::max(max_cost_, costs_[row][col]);
      }
    }
  }

  // Initially, none of the cells of the matrix are marked.
  marks_.resize(matrix_size_);
  for (int row = 0; row < matrix_size_; ++row) {
    marks_[row].resize(matrix_size_);
    for (int col = 0; col < matrix_size_; ++col) {
      marks_[row][col] = NONE;
    }
  }

  stars_in_col_.resize(matrix_size_);

  rows_covered_.resize(matrix_size_);
  cols_covered_.resize(matrix_size_);

  preimage_.resize(matrix_size_ * 2);
  image_.resize(matrix_size_ * 2);

  uncov_col_.resize(matrix_size_);
  uncov_row_.resize(matrix_size_);
}

// Find an assignment which maximizes the total cost.
// Return an array of pairs of integers.  Each pair (i, j) corresponds to
// assigning agent i to task j.
void HungarianOptimizer::maximize(std::vector<int>* preimage,
                                  std::vector<int>* image) {
  // Find a maximal assignment by subtracting each of the
  // original costs from max_cost_  and then minimizing.
  for (int row = 0; row < width_; ++row) {
    for (int col = 0; col < height_; ++col) {
      costs_[row][col] = max_cost_ - costs_[row][col];
    }
  }
  minimize(preimage, image);
}

// Find an assignment which minimizes the total cost.
// Return an array of pairs of integers.  Each pair (i, j) corresponds to
// assigning agent i to task j.
void HungarianOptimizer::minimize(std::vector<int>* preimage,
                                  std::vector<int>* image) {
  do_munkres();
  find_assignments(preimage, image);
}

// Convert the final cost matrix into a set of assignments of agents -> tasks.
// Return an array of pairs of integers, the same as the return values of
// Minimize() and Maximize()
void HungarianOptimizer::find_assignments(std::vector<int>* preimage,
                                          std::vector<int>* image) {
  preimage->clear();
  image->clear();
  for (int row = 0; row < width_; ++row) {
    for (int col = 0; col < height_; ++col) {
      if (is_starred(row, col)) {
        preimage->push_back(row);
        image->push_back(col);
        break;
      }
    }
  }
  // TODO(user)
  // result_size = std::min(width_, height_);
  // CHECK image.size() == result_size
  // CHECK preimage.size() == result_size
}

// Find a column in row 'row' containing a star, or return
// kHungarianOptimizerColNotFound if no such column exists.
int HungarianOptimizer::find_star_in_row(int row) const {
  for (int col = 0; col < matrix_size_; ++col) {
    if (is_starred(row, col)) {
      return col;
    }
  }

  return kHungarianOptimizerColNotFound;
}

// Find a row in column 'col' containing a star, or return
// kHungarianOptimizerRowNotFound if no such row exists.
int HungarianOptimizer::find_star_in_col(int col) const {
  if (!col_contains_star(col)) {
    return kHungarianOptimizerRowNotFound;
  }

  for (int row = 0; row < matrix_size_; ++row) {
    if (is_starred(row, col)) {
      return row;
    }
  }

  // NOTREACHED
  return kHungarianOptimizerRowNotFound;
}

// Find a column in row containing a prime, or return
// kHungarianOptimizerColNotFound if no such column exists.
int HungarianOptimizer::find_prime_in_row(int row) const {
  for (int col = 0; col < matrix_size_; ++col) {
    if (is_primed(row, col)) {
      return col;
    }
  }

  return kHungarianOptimizerColNotFound;
}

// Remove the prime marks from every cell in the matrix.
void HungarianOptimizer::clear_primes() {
  for (int row = 0; row < matrix_size_; ++row) {
    for (int col = 0; col < matrix_size_; ++col) {
      if (is_primed(row, col)) {
        marks_[row][col] = NONE;
      }
    }
  }
}

// Uncovery ever row and column in the matrix.
void HungarianOptimizer::clear_covers() {
  for (int x = 0; x < matrix_size_; x++) {
    uncover_row(x);
    uncover_col(x);
  }
}

// Find the smallest uncovered cell in the matrix.
double HungarianOptimizer::find_smallest_uncovered() {
  double minval = std::numeric_limits<double>::max();
  uncov_col_.clear();
  uncov_row_.clear();

  for (int i = 0; i < matrix_size_; ++i) {
    if (!row_covered(i)) {
      uncov_row_.push_back(i);
    }
    if (!col_covered(i)) {
      uncov_col_.push_back(i);
    }
  }

  for (size_t row = 0; row < uncov_row_.size(); ++row) {
    for (size_t col = 0; col < uncov_col_.size(); ++col) {
      minval = std::min(minval, costs_[uncov_row_[row]][uncov_col_[col]]);
    }
  }

  return minval;
}

// Find an uncovered zero and store its co-ordinates in (zeroRow, zeroCol)
// and return true, or return false if no such cell exists.
bool HungarianOptimizer::find_zero(int* zero_row, int* zero_col) {
  uncov_col_.clear();
  uncov_row_.clear();

  for (int i = 0; i < matrix_size_; ++i) {
    if (!row_covered(i)) {
      uncov_row_.push_back(i);
    }
    if (!col_covered(i)) {
      uncov_col_.push_back(i);
    }
  }
  if (uncov_row_.empty() || uncov_col_.empty()) {
    return false;
  }

  for (size_t i = 0; i < uncov_row_.size(); ++i) {
    for (size_t j = 0; j < uncov_col_.size(); ++j) {
      if (costs_[uncov_row_[i]][uncov_col_[j]] == 0) {
        *zero_row = uncov_row_[i];
        *zero_col = uncov_col_[j];
        return true;
      }
    }
  }
  return false;
}

// Print the matrix to stdout (for debugging.)
void HungarianOptimizer::print_matrix() {
  for (int row = 0; row < matrix_size_; ++row) {
    for (int col = 0; col < matrix_size_; ++col) {
      printf("%g ", costs_[row][col]);

      if (is_starred(row, col)) {
        printf("*");
      }

      if (is_primed(row, col)) {
        printf("'");
      }
    }
    printf("\n");
  }
}

//  Run the Munkres algorithm!
void HungarianOptimizer::do_munkres() {
  int max_iter = 1000;
  int iter_num = 0;
  state_ = &HungarianOptimizer::reduce_rows;
  while (state_ != NULL && iter_num < max_iter) {
    // while (state_ != NULL) {
    (this->*state_)();
    ++iter_num;
  }
  if (iter_num >= max_iter) {
    check_star();
  }
}

void HungarianOptimizer::check_star() {
  for (int row = 0; row < width_; ++row) {
    int star_col = -1;
    bool is_single = true;
    for (int col = 0; col < height_; ++col) {
      if (is_starred(row, col)) {
        if (star_col == -1) {
          star_col = col;
        } else {
          is_single = false;
          break;
        }
      }
    }
    if (!is_single) {
      for (int col = 0; col < height_; ++col) {
        unstar(row, col);
      }
    }
  }
}
// Step 1.
// For each row of the matrix, find the smallest element and subtract it
// from every element in its row.  Go to Step 2.
void HungarianOptimizer::reduce_rows() {
  for (int row = 0; row < matrix_size_; ++row) {
    double min_cost = costs_[row][0];
    for (int col = 1; col < matrix_size_; ++col) {
      min_cost = std::min(min_cost, costs_[row][col]);
    }
    for (int col = 0; col < matrix_size_; ++col) {
      costs_[row][col] -= min_cost;
    }
  }
  state_ = &HungarianOptimizer::star_zeroes;
}

// Step 2.
// Find a zero (Z) in the matrix.  If there is no starred zero in its row
// or column, star Z.  Repeat for every element in the matrix.  Go to step 3.
// of the CPU - the next slowest step takes 0.6%.  I can't think of a way
// of speeding it up though.
void HungarianOptimizer::star_zeroes() {
  // Since no rows or columns are covered on entry to this step, we use the
  // covers as a quick way of marking which rows & columns have stars in them.
  for (int row = 0; row < matrix_size_; ++row) {
    if (row_covered(row)) {
      continue;
    }

    for (int col = 0; col < matrix_size_; ++col) {
      if (col_covered(col)) {
        continue;
      }

      if (costs_[row][col] == 0) {
        star(row, col);
        cover_row(row);
        cover_col(col);
        break;
      }
    }
  }

  clear_covers();
  state_ = &HungarianOptimizer::cover_starred_zeroes;
}

// Step 3.
// Cover each column containing a starred zero.  If all columns are
// covered, the starred zeros describe a complete set of unique assignments.
// In this case, terminate the algorithm.  Otherwise, go to step 4.
void HungarianOptimizer::cover_starred_zeroes() {
  int num_covered = 0;

  for (int col = 0; col < matrix_size_; ++col) {
    if (col_contains_star(col)) {
      cover_col(col);
      num_covered++;
    }
  }

  if (num_covered >= matrix_size_) {
    state_ = NULL;
    return;
  }
  state_ = &HungarianOptimizer::prime_zeroes;
}

// Step 4.
// Find a noncovered zero and prime it.  If there is no starred zero in the
// row containing this primed zero, Go to Step 5.  Otherwise, cover this row
// and uncover the column containing the starred zero. Continue in this manner
// until there are no uncovered zeros left, then go to Step 6.

void HungarianOptimizer::prime_zeroes() {
  // This loop is guaranteed to terminate in at most matrix_size_ iterations,
  // as find_zero() returns a location only if there is at least one uncovered
  // zero in the matrix.  Each iteration, either one row is covered or the
  // loop terminates.  Since there are matrix_size_ rows, after that many
  // iterations there are no uncovered cells and hence no uncovered zeroes,
  // so the loop terminates.
  for (;;) {
    int zero_row = 0;
    int zero_col = 0;
    if (!find_zero(&zero_row, &zero_col)) {
      // No uncovered zeroes.
      state_ = &HungarianOptimizer::augment_path;
      return;
    }

    prime(zero_row, zero_col);
    int star_col = find_star_in_row(zero_row);

    if (star_col != kHungarianOptimizerColNotFound) {
      cover_row(zero_row);
      uncover_col(star_col);
    } else {
      preimage_[0] = zero_row;
      image_[0] = zero_col;
      state_ = &HungarianOptimizer::make_augmenting_path;
      return;
    }
  }
}

// Step 5.
// Construct a series of alternating primed and starred zeros as follows.
// Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote
// the starred zero in the column of Z0 (if any). Let Z2 denote the primed
// zero in the row of Z1 (there will always be one).  Continue until the
// series terminates at a primed zero that has no starred zero in its column.
// unstar each starred zero of the series, star each primed zero of the
// series, erase all primes and uncover every line in the matrix.  Return to
// Step 3.
void HungarianOptimizer::make_augmenting_path() {
  bool done = false;
  int count = 0;

  /* Note: this loop is guaranteed to terminate within matrix_size_ iterations
  // because:
  // 1) on entry to this step, there is at least 1 column with no starred zero
  //    (otherwise we would have terminated the algorithm already.)
  // 2) each row containing a star also contains exactly one primed zero.
  // 4) each column contains at most one starred zero.
  //
  // Since the path_ we construct visits primed and starred zeroes alternately,
  // and terminates if we reach a primed zero in a column with no star, our
  // path_ must either contain matrix_size_ or fewer stars (in which case the
  // loop iterates fewer than matrix_size_ times), or it contains more.  In
  // that case, because (1) implies that there are fewer than
  // matrix_size_ stars, we must have visited at least one star more than once.
  // Consider the first such star that we visit more than once; it must have
  // been reached immediately after visiting a prime in the same row.  By (2),
  // this prime is unique and so must have also been visited more than once.
  // Therefore, that prime must be in the same column as a star that has been
  // visited more than once, contradicting the assumption that we chose the
  // first multiply visited star, or it must be in the same column as more
  // than one star, contradicting (3).  Therefore, we never visit any star
  // more than once and the loop terminates within matrix_size_ iterations.
  */

  while (!done) {
    // First construct the alternating path...
    int row = find_star_in_col(image_[count]);

    if (row != kHungarianOptimizerRowNotFound) {
      count++;
      preimage_[count] = row;
      image_[count] = image_[count - 1];
    } else {
      done = true;
    }

    if (!done) {
      int col = find_prime_in_row(preimage_[count]);
      count++;
      preimage_[count] = preimage_[count - 1];
      image_[count] = col;
    }
  }

  // Then modify it.
  for (int i = 0; i <= count; ++i) {
    int row = preimage_[i];
    int col = image_[i];

    if (is_starred(row, col)) {
      unstar(row, col);
    } else {
      star(row, col);
    }
  }

  clear_covers();
  clear_primes();
  state_ = &HungarianOptimizer::cover_starred_zeroes;
}

// Step 6
// Add the smallest uncovered value in the matrix to every element of each
// covered row, and subtract it from every element of each uncovered column.
// Return to Step 4 without altering any stars, primes, or covered lines.
void HungarianOptimizer::augment_path() {
  double minval = find_smallest_uncovered();

  for (int row = 0; row < matrix_size_; ++row) {
    if (row_covered(row)) {
      for (int c = 0; c < matrix_size_; ++c) {
        costs_[row][c] += minval;
      }
    }
  }
  for (int col = 0; col < matrix_size_; ++col) {
    if (!col_covered(col)) {
      for (int r = 0; r < matrix_size_; ++r) {
        costs_[r][col] -= minval;
      }
    }
  }
  state_ = &HungarianOptimizer::prime_zeroes;
}

}  // namespace perception
}  // namespace apollo
