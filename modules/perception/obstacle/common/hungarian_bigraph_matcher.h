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

/***************************************************************************** 
* Copyright 2010-2012 Google
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
* See: //depot/or-tools/java/com/google/wireless/genie/frontend
* /mixer/matching/HungarianOptimizer.java
****************************************************************************/

#ifndef  MODULES_PERCEPTION_OBSTACLE_COMMON_HUNGARIAN_BIGRAPH_MATCHER_H_
#define  MODULES_PERCEPTION_OBSTACLE_COMMON_HUNGARIAN_BIGRAPH_MATCHER_H_

#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

namespace apollo {
namespace perception {

class HungarianOptimizer {
  static const int kHungarianOptimizerRowNotFound = -1;
  static const int kHungarianOptimizerColNotFound = -2;

 public:
  // Setup the initial conditions for the algorithm.

  // Parameters: costs is a matrix of the cost of assigning each agent to
  // each task. costs[i][j] is the cost of assigning agent i to task j.
  // All the costs must be non-negative.  This matrix does not have to
  // be square (i.e. we can have different numbers of agents and tasks), but it
  // must be regular (i.e. there must be the same number of entries in each row
  // of the matrix).
  explicit HungarianOptimizer(const std::vector<std::vector<double> >& costs);

  // Find an assignment which maximizes the total cost.
  // Returns the assignment in the two vectors passed as argument.
  // agent[i] is assigned to task[i].
  void maximize(std::vector<int>* agent, std::vector<int>* task);

  // Find an assignment which minimizes the total cost.
  // Returns the assignment in the two vectors passed as argument.
  // agent[i] is assigned to task[i].
  void minimize(std::vector<int>* agent, std::vector<int>* task);

 private:
  typedef void (HungarianOptimizer::*Step)();

  typedef enum {
    NONE,
    PRIME,
    STAR
  } Mark;

  // Convert the final cost matrix into a set of assignments of agents -> tasks.
  // Returns the assignment in the two vectors passed as argument, the same as
  // Minimize and Maximize
  void find_assignments(std::vector<int>* agent, std::vector<int>* task);

  // Is the cell (row, col) starred?
  bool is_starred(int row, int col) const {
    return _marks[row][col] == STAR;
  }

  // Mark cell (row, col) with a star
  void star(int row, int col) {
    _marks[row][col] = STAR;
    _stars_in_col[col]++;
  }

  // Remove a star from cell (row, col)
  void unstar(int row, int col) {
    _marks[row][col] = NONE;
    _stars_in_col[col]--;
  }

  // Find a column in row 'row' containing a star, or return
  // kHungarianOptimizerColNotFound if no such column exists.
  int find_star_in_row(int row) const;

  // Find a row in column 'col' containing a star, or return
  // kHungarianOptimizerRowNotFound if no such row exists.
  int find_star_in_col(int col) const;

  // Is cell (row, col) marked with a prime?
  bool is_primed(int row, int col) const {
    return _marks[row][col] == PRIME;
  }

  // Mark cell (row, col) with a prime.
  void prime(int row, int col) {
    _marks[row][col] = PRIME;
  }

  // Find a column in row containing a prime, or return
  // kHungarianOptimizerColNotFound if no such column exists.
  int find_prime_in_row(int row) const;

  // Remove the prime _marks from every cell in the matrix.
  void clear_primes();

  // Does column col contain a star?
  bool col_contains_star(int col) const {
    return _stars_in_col[col] > 0;
  }

  // Is row 'row' covered?
  bool row_covered(int row) const {
    return _rows_covered[row];
  }

  // Cover row 'row'.
  void cover_row(int row) {
    _rows_covered[row] = true;
  }

  // Uncover row 'row'.
  void uncover_row(int row) {
    _rows_covered[row] = false;
  }

  // Is column col covered?
  bool col_covered(int col) const {
    return _cols_covered[col];
  }

  // Cover column col.
  void cover_col(int col) {
    _cols_covered[col] = true;
  }

  // Uncover column col.
  void uncover_col(int col) {
    _cols_covered[col] = false;
  }

  // Uncover ever row and column in the matrix.
  void clear_covers();

  // Find the smallest uncovered cell in the matrix.
  double find_smallest_uncovered();

  // Find an uncovered zero and store its coordinates in (zeroRow_, zeroCol_)
  // and return true, or return false if no such cell exists.
  bool find_zero(int* zero_row, int* zero_col);

  // Print the matrix to stdout (for debugging.)
  void print_matrix();

  // Run the Munkres algorithm!
  void do_munkres();

  void check_star();

  // Step 1.
  // For each row of the matrix, find the smallest element and subtract it
  // from every element in its row.  Go to Step 2.
  void reduce_rows();

  // Step 2.
  // Find a zero (Z) in the matrix.  If there is no starred zero in its row
  // or column, star Z.  Repeat for every element in the matrix.  Go to step 3.
  // Note: profiling shows this method to use 9.2% of the CPU - the next
  // slowest step takes 0.6%.  I can't think of a way of speeding it up though.
  void star_zeroes();

  // Step 3.
  // Cover each column containing a starred zero.  If all columns are
  // covered, the starred zeros describe a complete set of unique assignments.
  // In this case, terminate the algorithm.  Otherwise, go to step 4.
  void cover_starred_zeroes();

  // Step 4.
  // Find a noncovered zero and prime it.  If there is no starred zero in the
  // row containing this primed zero, Go to Step 5.  Otherwise, cover this row
  // and uncover the column containing the starred zero. Continue in this manner
  // until there are no uncovered zeros left, then go to Step 6.
  void prime_zeroes();

  // Step 5.
  // Construct a series of alternating primed and starred zeros as follows.
  // Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote
  // the starred zero in the column of Z0 (if any). Let Z2 denote the primed
  // zero in the row of Z1 (there will always be one).  Continue until the
  // series terminates at a primed zero that has no starred zero in its column.
  // Unstar each starred zero of the series, star each primed zero of the
  // series, erase all primes and uncover every line in the matrix.  Return to
  // Step 3.
  void make_augmenting_path();

  // Step 6.
  // Add the smallest uncovered value in the matrix to every element of each
  // covered row, and subtract it from every element of each uncovered column.
  // Return to Step 4 without altering any stars, primes, or covered lines.
  void augment_path();

  // The size of the problem, i.e. std::max(#agents, #tasks).
  int _matrix_size;

  // The expanded cost matrix.
  std::vector<std::vector<double> > _costs;

  // The greatest cost in the initial cost matrix.
  double _max_cost;

  // Which rows and columns are currently covered.
  std::vector<bool> _rows_covered;
  std::vector<bool> _cols_covered;

  // The _marks (star/prime/none) on each element of the cost matrix.
  std::vector<std::vector<Mark> > _marks;

  // The number of stars in each column - used to speed up coverStarredZeroes.
  std::vector<int> _stars_in_col;

  // Representation of a path_ through the matrix - used in step 5.
  std::vector<int> _preimage;  // i.e. the agents
  std::vector<int> _image;     // i.e. the tasks

  // The locations of a zero found in step 4.
  int _zero_col;
  int _zero_row;

  // The _width and _height of the initial (non-expanded) cost matrix.
  int _width;
  int _height;

  // The current state of the algorithm
  HungarianOptimizer::Step _state;

  std::vector<int> _uncov_col;
  std::vector<int> _uncov_row;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_HUNGARIAN_BIGRAPH_MATCHER_H_
