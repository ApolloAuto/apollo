
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

#include <cstddef>
#include <iostream>

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

template <typename Scalar, int aligned_len = alignof(max_align_t)>
class AlignedMatrix {
 public:
  AlignedMatrix();
  explicit AlignedMatrix(const AlignedMatrix<Scalar, aligned_len>& matrix);
  ~AlignedMatrix();

  void Init(int rows, int cols);
  void MakeEmpty();
  void MakeEmpty(int start_id, int end_id);
  int GetRow() const;
  int GetCol() const;
  Scalar* GetData();
  void SetData(const Scalar* data, unsigned int data_size,
               unsigned int start_id);

  AlignedMatrix& operator=(const AlignedMatrix<Scalar, aligned_len>& matrix);

  inline Scalar* operator[](int row) { return row_data_[row]; }

  inline const Scalar* operator[](int row) const { return row_data_[row]; }

 protected:
  Scalar* data_ = nullptr;
  Scalar** row_data_ = nullptr;
  int rows_ = 0;
  int cols_ = 0;

 private:
  void* raw_ptr_ = nullptr;
  int raw_size_ = 0;
};

template <typename Scalar, int aligned_len>
AlignedMatrix<Scalar, aligned_len>::AlignedMatrix()
    : data_(nullptr),
      row_data_(nullptr),
      rows_(0),
      cols_(0),
      raw_ptr_(nullptr),
      raw_size_(0) {}

template <typename Scalar, int aligned_len>
AlignedMatrix<Scalar, aligned_len>::AlignedMatrix(
    const AlignedMatrix<Scalar, aligned_len>& matrix) {
  rows_ = matrix.rows_;
  cols_ = matrix.cols_;

  raw_size_ = matrix.raw_size_;
  raw_ptr_ = malloc(raw_size_);

  row_data_ = reinterpret_cast<Scalar**>(malloc(sizeof(Scalar*) * rows_));

  unsigned char* ptr = reinterpret_cast<unsigned char*>(raw_ptr_);
  int idx = 0;
  while (idx < aligned_len) {
    if ((uint64_t)(ptr) % aligned_len == 0) {
      break;
    }
    ++ptr;
    ++idx;
  }
  data_ = reinterpret_cast<Scalar*>(ptr);

  for (unsigned int k = 0; k < rows_; k++) {
    row_data_[k] = data_ + k * cols_;
  }

  memcpy(data_, matrix.data_, sizeof(Scalar) * rows_ * cols_);
}

template <typename Scalar, int aligned_len>
AlignedMatrix<Scalar, aligned_len>::~AlignedMatrix() {
  if (raw_ptr_) {
    free(raw_ptr_);
    raw_size_ = 0;
    raw_ptr_ = nullptr;
  }

  if (row_data_) {
    free(row_data_);
    row_data_ = nullptr;
  }

  data_ = nullptr;
  raw_size_ = 0;
}

template <typename Scalar, int aligned_len>
void AlignedMatrix<Scalar, aligned_len>::Init(int rows, int cols) {
  if (raw_ptr_) {
    free(raw_ptr_);
    raw_size_ = 0;
    raw_ptr_ = nullptr;
  }

  if (row_data_) {
    free(row_data_);
    row_data_ = nullptr;
  }

  rows_ = rows;
  cols_ = cols;

  raw_size_ = static_cast<int>(sizeof(Scalar)) * (rows * cols) + aligned_len;
  raw_ptr_ = malloc(raw_size_);

  row_data_ = reinterpret_cast<Scalar**>(malloc(sizeof(Scalar*) * rows_));

  unsigned char* ptr = reinterpret_cast<unsigned char*>(raw_ptr_);
  int idx = 0;
  while (idx < aligned_len) {
    if ((uint64_t)(ptr) % aligned_len == 0) {
      break;
    }
    ++ptr;
    ++idx;
  }
  data_ = reinterpret_cast<Scalar*>(ptr);

  for (unsigned int k = 0; k < rows_; k++) {
    row_data_[k] = data_ + k * cols_;
  }

  MakeEmpty();

  // printf("aligned addr: %p\n", (void*)data_);
}

template <typename Scalar, int aligned_len>
void AlignedMatrix<Scalar, aligned_len>::MakeEmpty() {
  memset(data_, 0, sizeof(Scalar) * rows_ * cols_);
}

template <typename Scalar, int aligned_len>
void AlignedMatrix<Scalar, aligned_len>::MakeEmpty(int start_id, int end_id) {
  memset(data_ + start_id, 0, sizeof(Scalar) * (end_id - start_id + 1));
}

template <typename Scalar, int aligned_len>
int AlignedMatrix<Scalar, aligned_len>::GetRow() const {
  return rows_;
}

template <typename Scalar, int aligned_len>
int AlignedMatrix<Scalar, aligned_len>::GetCol() const {
  return cols_;
}

template <typename Scalar, int aligned_len>
Scalar* AlignedMatrix<Scalar, aligned_len>::GetData() {
  return data_;
}

template <typename Scalar, int aligned_len>
void AlignedMatrix<Scalar, aligned_len>::SetData(const Scalar* data,
                                                 unsigned int data_size,
                                                 unsigned int start_id) {
  memcpy(data_ + start_id, data, sizeof(Scalar) * data_size);
}

template <typename Scalar, int aligned_len>
AlignedMatrix<Scalar, aligned_len>& AlignedMatrix<Scalar, aligned_len>::
operator=(const AlignedMatrix<Scalar, aligned_len>& matrix) {
  if (raw_ptr_) {
    free(raw_ptr_);
    raw_size_ = 0;
    raw_ptr_ = nullptr;
  }

  if (row_data_) {
    free(row_data_);
    row_data_ = nullptr;
  }

  rows_ = matrix.rows_;
  cols_ = matrix.cols_;

  raw_size_ = matrix.raw_size_;
  raw_ptr_ = malloc(raw_size_);

  row_data_ = reinterpret_cast<Scalar**>(malloc(sizeof(Scalar*) * rows_));

  unsigned char* ptr = reinterpret_cast<unsigned char*>(raw_ptr_);
  int idx = 0;
  while (idx < aligned_len) {
    if ((uint64_t)(ptr) % aligned_len == 0) {
      break;
    }
    ++ptr;
    ++idx;
  }
  data_ = reinterpret_cast<Scalar*>(ptr);

  for (unsigned int k = 0; k < rows_; k++) {
    row_data_[k] = data_ + k * cols_;
  }

  memcpy(data_, matrix.data_, sizeof(Scalar) * rows_ * cols_);
  return *this;
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
