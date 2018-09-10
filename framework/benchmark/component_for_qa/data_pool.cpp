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

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>

#include "data_pool.h"
#include "cybertron/cybertron.h"

DataPool::DataPool(int length, int interval, int step)
    : _data(NULL),
      _length(length),
      _interval(interval),
      _step(step),
      _count(0) {
  init_pool();
}

DataPool::~DataPool() {
  std::cout << "=====================~DataPool::~DataPool==================="
            << std::endl;
  if (_data != NULL) {
    delete _data;
  }
}

int DataPool::get_total() { return _length + _interval; }

const char* DataPool::get_data(const int pos) { return _data + pos; }

void DataPool::init_pool() {
  char init_chars[128];
  for (int i = 1; i <= 128; i++) {
    init_chars[i - 1] = (char)i;
  }

  _total = _length + _interval;
  _data = new char[DATA_POOL_SIZE + 1];
  // TODO: memcpy
  for (int i = 0; i < DATA_POOL_SIZE; i++) {
    _data[i] = init_chars[i & 127];
  }
  _data[DATA_POOL_SIZE] = '\0';
}

char* DataPool::allocate() {
  int total = get_total() + 1;
  char* allocating = new char[total + 1];
  for (int i = 0; i < total + 1; i++) {
    allocating[i] = '\0';
  }
  return allocating;
}

int DataPool::allocate(char* allocating, int& position) {
  int length = _length;
  int total = _length + _interval;
  // need to random data length?
  if (position >= total) {
    position = 0;
  }
  if (_interval > 0 && position > 0) {
    length = length + (_interval - rand() % _interval) / _step * _step;
    if (length <= 0) {
      length = _interval;
    }
  }
  std::cout << "length: " << length << std::endl;
  // int left_count = std::min(total - position, length);
  // firt part, but the backward of pool
  // snprintf(allocating, left_count + 1, "%s", get_data(position));
  // second part, but the front of pool
  // snprintf(allocating + left_count, length - left_count + 1, "%s",
  // get_data(0));
  // total
  auto start = apollo::cybertron::Time::Now().ToNanosecond();
  // int written = snprintf(allocating, length + 1, "%s\0", get_data(position));
  strncpy(allocating, get_data(position), length);
  allocating[length] = '\0';
  // std::cout << "^^^^^^^ " << ", length: " << length << ", written: " <<
  // written << std::endl;
  if (strlen(allocating) == 0) {
    std::cout << "========0 " << strlen(get_data(position))
              //<< ", length: " << length << ", written: " << written
              << std::endl;
  }
  auto end = apollo::cybertron::Time::Now().ToNanosecond();
  std::cout << "allocate snprintf use: " << start << " " << end << " "
            << apollo::cybertron::Time::Now().ToNanosecond() - start << std::endl;
  /*
  std::cout << "-------------------------" << std::endl;
  if (length < 30) {
      for (int i = 0; i < length; i++) {
          std::cout << (int)allocating[i] << std::endl;
      }
  }
  std::cout << "-------------------------" << std::endl;
  std::cout << "[writer]: "
            << "length: " << length << ", interval: " << _interval
            << ", position: " << position
            << ", data length: " << strlen(allocating) << std::endl;
  */
  return length;
}

bool DataPool::compare(const char* comparatee, int length, int position) {
  int total = _length + _interval;
  char data[length + 1];
  // snprintf(data, length + 1, "%s", get_data(position));
  // data[length] = '\0';
  // int left_count = std::min(total - position, length);
  // firt part, but the backward of pool
  // int writen1 = snprintf(allocating, left_count + 1, "%s",
  // get_data(position));
  // second part, but the front of pool
  // int writen2 = snprintf(allocating + left_count, length - left_count + 1,
  // "%s", get_data(0));
  // allocating[length] = '\0';
  /*
  std::cout << "[reader]: "
            << "total: " << total << ", length: " << length
            << ", position: " << position << std::endl;
  std::cout << "******************" << std::endl;
  for (int i = 0; i < length; i++) {
      std::cout << (int)data[i] << ' ' << (int)comparatee[i] << std::endl;
  }
  std::cout << "******************" << std::endl;
  std::cout << "data: " << data << "\ncomparatee: " << comparatee << std::endl;
  */
  //return strncmp(comparatee, get_data(position), length) == 0;
  return true;
}

