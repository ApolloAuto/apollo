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

#ifndef CYBERTRON_BENCHMARK_COMPONENT_TEST_DATA_POOL_H
#define CYBERTRON_BENCHMARK_COMPONENT_TEST_DATA_POOL_H

class DataPool {
 public:
  DataPool(int length, int interval, int step);
  ~DataPool();
  const char* get_data(const int pos);
  int allocate(char* allocating, int& position);
  bool compare(const char* comparatee, int length, int position);
  char* allocate();
  int get_total();

 private:
  DataPool(const DataPool&);

  void init_pool();
  char* _data;
  int _total;
  int _length;
  int _interval;
  int _step;
  int _count;
  const int DATA_POOL_SIZE = 35 * 1024 * 1024;
};

#endif // CYBERTRON_BENCHMARK_COMPONENT_TEST_DATA_POOL_H
