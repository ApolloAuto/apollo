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

#ifndef TOOLS_CVT_VPERF_TIMEBLOCKITEMPOOL_H_
#define TOOLS_CVT_VPERF_TIMEBLOCKITEMPOOL_H_

#include <memory>
#include <vector>

class TimeBlockItem;

// no thread safety
class TimeBlockItemPool {
 public:
  struct SmallPool;
  static constexpr uint SmallPoolBlockCount = 2000;
  static TimeBlockItemPool* instance(void);
  ~TimeBlockItemPool(void) { destroy(); }

  void destroy(void);
  void reset(void);
  bool extend(int newBlockCount);

  TimeBlockItem* allocate(void);
  void deallocate(TimeBlockItem* item);

 private:
  TimeBlockItemPool() : _poolHeaders() {}
  bool extend(void);

  std::vector<SmallPool*> _poolHeaders;
};

#endif  // TIMEBLOCKITEMPOOL_H
