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

#include <boost/filesystem.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/tool/benchmark/lidar/loader/string_compare.h"

namespace apollo {
namespace perception {
namespace benchmark {

// @brief General data loader base class, class DataType must implement
// "bool load(const std::vector<std::string>& filenames) and void release()"
// member function
template <class DataType>
class SequenceDataLoader {
 public:
  SequenceDataLoader() = default;
  virtual ~SequenceDataLoader() = default;
  bool init_loader_with_list(const std::vector<std::string>& file_lists);
  bool init_loader_with_folder(const std::vector<std::string>& folders);
  virtual bool query_next(std::shared_ptr<DataType>& data);  // NOLINT
  virtual bool query_last(std::shared_ptr<DataType>& data);  // NOLINT
  inline std::size_t size() { return _initialized ? _filenames[0].size() : 0; }
  inline const std::vector<std::vector<std::string>>& get_filenames() const {
    return _filenames;
  }

 protected:
  std::vector<std::vector<std::string>> _filenames;
  int _idx = -1;
  bool _initialized = false;
};

template <class DataType>
bool SequenceDataLoader<DataType>::init_loader_with_list(
    const std::vector<std::string>& file_lists) {
  _filenames.clear();
  _filenames.resize(file_lists.size());

  std::ifstream fin;
  std::string name = "";
  std::vector<std::size_t> sorted_indices;
  for (std::size_t i = 0; i < file_lists.size(); ++i) {
    fin.open(file_lists[i].c_str());
    if (!fin.is_open()) {
      _initialized = false;
      return false;
    }
    std::getline(fin, name);
    while (!fin.eof()) {
      _filenames[i].push_back(name);
      std::getline(fin, name);
    }
    fin.close();
  }
  sort_strings_by_split_length(_filenames[0], &sorted_indices);
  shuffle_by_indices(&_filenames[0], sorted_indices);

  for (std::size_t i = 1; i < file_lists.size(); ++i) {
    if (_filenames[i].size() != _filenames[0].size()) {
      _initialized = false;
      return false;
    }
    shuffle_by_indices(&_filenames[i], sorted_indices);
  }
  _idx = -1;
  _initialized = true;
  return true;
}

template <class DataType>
bool SequenceDataLoader<DataType>::init_loader_with_folder(
    const std::vector<std::string>& folders) {
  _filenames.clear();
  _filenames.resize(folders.size());

  for (std::size_t i = 0; i < folders.size(); ++i) {
    if (!boost::filesystem::exists(folders[i])) {
      _initialized = false;
      return false;
    }
    boost::filesystem::directory_iterator it(folders[i]);
    boost::filesystem::directory_iterator eit;
    while (it != eit) {
      _filenames[i].push_back(it->path().string());
      ++it;
    }
    std::sort(_filenames[i].begin(), _filenames[i].end(),
              string_compare_by_length);
  }
  for (std::size_t i = 1; i < folders.size(); ++i) {
    if (_filenames[i].size() != _filenames[0].size()) {
      _initialized = false;
      return false;
    }
  }
  _idx = -1;
  _initialized = true;
  return true;
}

template <class DataType>
bool SequenceDataLoader<DataType>::query_next(
    std::shared_ptr<DataType>& data) {  // NOLINT
  if (!_initialized) {
    return false;
  }
  if (data == nullptr) {
    data.reset(new DataType);
  }
  ++_idx;
  if (_idx >= static_cast<int>(_filenames[0].size())) {
    return false;
  } else if (_idx < 0) {
    _idx = 0;
  }
  std::vector<std::string> files;
  for (auto& names : _filenames) {
    files.push_back(names[_idx]);
  }
  return data->load(files);
}

template <class DataType>
bool SequenceDataLoader<DataType>::query_last(
    std::shared_ptr<DataType>& data) {  // NOLINT
  if (!_initialized) {
    return false;
  }
  if (data == nullptr) {
    data.reset(new DataType);
  }
  --_idx;
  if (_idx < 0) {
    return false;
  } else if (_idx >= static_cast<int>(_filenames[0].size())) {
    _idx = static_cast<int>(_filenames[0].size() - 1);
  }
  std::vector<std::string> files;
  for (auto& names : _filenames) {
    files.push_back(names[_idx]);
  }
  return data->load(files);
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
