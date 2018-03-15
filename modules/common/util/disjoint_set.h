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

#ifndef MODULES_COMMON_UTIL_DISJOINT_SET_H_
#define MODULES_COMMON_UTIL_DISJOINT_SET_H_

namespace apollo {
namespace common {
namespace util {

template <class T>
void DisjointSetMakeSet(T *x) {
  x->parent = x;
  x->node_rank = 0;
}

template <class T>
T *DisjointSetFindRecursive(T *x) {
  if (x->parent != x) {
    x->parent = DisjointSetFindRecursive(x->parent);
  }
  return x->parent;
}

template <class T>
T *DisjointSetFind(T *x) {
  T *y = x->parent;
  if (y == x || y->parent == y) {
    return y;
  }
  T *root = DisjointSetFindRecursive(y->parent);
  x->parent = root;
  y->parent = root;
  return root;
}

template <class T>
void DisjointSetMerge(T *x, const T *y) {}

template <class T>
void DisjointSetUnion(T *x, T *y) {
  x = DisjointSetFind(x);
  y = DisjointSetFind(y);
  if (x == y) {
    return;
  }
  if (x->node_rank < y->node_rank) {
    x->parent = y;
    DisjointSetMerge(y, x);
  } else if (y->node_rank < x->node_rank) {
    y->parent = x;
    DisjointSetMerge(x, y);
  } else {
    y->parent = x;
    x->node_rank++;
    DisjointSetMerge(x, y);
  }
}

}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_PERCEPTION_COMMON_UTIL_DISJOINT_SET_H_
