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

#include "time_block_item_pool.h"
#include <iostream>
#include "time_block_item.h"

#define MEMBER_OFFSET(StructType, Member) (size_t) & (((StructType*)0)->Member)
#define StructPtrByMemberPtr(MemberPtr, StructType, Member) \
  (StructType*)((char*)MemberPtr - MEMBER_OFFSET(StructType, Member))

namespace {
struct Warpper {
  TimeBlockItemPool::SmallPool* _poolPtr;
  TimeBlockItem _timeBlockItem;

  explicit Warpper() : _poolPtr(nullptr), _timeBlockItem() {}
  ~Warpper() { _poolPtr = nullptr; }
};

struct SmallPoolData {
  union {
    SmallPoolData* _next;
    Warpper _warpperItem;
  };

  explicit SmallPoolData() : _warpperItem() { _next = nullptr; }

  ~SmallPoolData() { _warpperItem.~Warpper(); }
};
}

struct TimeBlockItemPool::SmallPool {
  int _freeCount;
  SmallPoolData* _freeList;

  explicit SmallPool() : _freeCount(0), _freeList(nullptr) {}

  TimeBlockItem* allocate(void) {
    --_freeCount;
    SmallPoolData* p = _freeList;
    _freeList = _freeList->_next;

    p->_warpperItem._poolPtr = this;

    return &(p->_warpperItem._timeBlockItem);
  }

  void deallocate(SmallPoolData* p) {
    p->_next = _freeList;
    _freeList = p;
    ++_freeCount;
  }
};

namespace {
struct SmallPoolWithData {  // don't change the member order
  TimeBlockItemPool::SmallPool _pool;
  SmallPoolData _data[TimeBlockItemPool::SmallPoolBlockCount];

  explicit SmallPoolWithData(void) : _pool(), _data() {}

  void reset(void) {
    SmallPoolData* cur = _data;  // i = 0

    _pool._freeCount = TimeBlockItemPool::SmallPoolBlockCount;
    _pool._freeList = cur;

    for (uint i = 1; i < TimeBlockItemPool::SmallPoolBlockCount; ++i) {
      cur->_next = _data + i;
      cur = _data + i;
    }

    cur->_next = nullptr;
  }
};
}

TimeBlockItemPool* TimeBlockItemPool::instance() {
  static TimeBlockItemPool pool;
  return &pool;
}

bool TimeBlockItemPool::extend(void) {
  SmallPoolWithData* newOne = new SmallPoolWithData();
  if (newOne == nullptr) return false;

  newOne->reset();

  _poolHeaders.push_back(&newOne->_pool);
  return true;
}

void TimeBlockItemPool::destroy() {
  for (SmallPool* item : _poolHeaders) {
    delete item;
  }
  _poolHeaders.clear();
}

void TimeBlockItemPool::reset(void) {
  for (SmallPool* pool : _poolHeaders) {
    SmallPoolWithData* pd = reinterpret_cast<SmallPoolWithData*>(pool);
    pd->reset();
  }
}

bool TimeBlockItemPool::extend(int newBlockCount) {
  newBlockCount += SmallPoolBlockCount - 1;
  newBlockCount /= SmallPoolBlockCount;

  int c = _poolHeaders.size();
  int i = c;
  newBlockCount += i;
  for (; i < newBlockCount; ++i) {
    if (!extend()) {
      break;
    }
  }

  if (i != newBlockCount) {
    for (; i > c; --i) {
      SmallPool* p = _poolHeaders.at(i - 1);
      _poolHeaders.pop_back();

      delete p;
    }
    return false;
  } else {
    return true;
  }
}

TimeBlockItem* TimeBlockItemPool::allocate(void) {
  for (SmallPool* p : _poolHeaders) {
    if (p->_freeCount) {
      return p->allocate();
    }
  }

  if (extend()) {
    SmallPool* p = _poolHeaders.at(_poolHeaders.size() - 1);
    return p->allocate();
  }

  return nullptr;
}

void TimeBlockItemPool::deallocate(TimeBlockItem* item) {
  if (item == nullptr) return;
  Warpper* w = StructPtrByMemberPtr(item, ::Warpper, _timeBlockItem);
  SmallPool* p = w->_poolPtr;

  p->deallocate((SmallPoolData*)(w));
}
