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

#ifndef CYBERTRON_SCHEDULER_POLICY_RBTREE_H_
#define CYBERTRON_SCHEDULER_POLICY_RBTREE_H_

#include <stdint.h>

namespace apollo {
namespace cybertron {
namespace scheduler {

class RBNode {
 public:
  uint64_t parent_color_;
  RBNode *right_;
  RBNode *left_;

  virtual bool Compare(RBNode *) = 0;
  virtual int Compare(void *) = 0;
  virtual void Print() = 0;
};

class RBTree {
 public:
  void Insert(RBNode *);
  void Delete(RBNode *);
  RBNode *Find(void *);
  RBNode *First();
  void Preorder();
  void Preorder(RBNode *);

 private:
  void InsertRebalance(RBNode *);
  void DeleteRebalance(RBNode *, RBNode *);
  void LinkNode(RBNode **, RBNode *, RBNode *);
  void RotateLeft(RBNode *);
  void RotateRight(RBNode *);

  RBNode *root_ = nullptr;
  // cache leftmost
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_POLICY_RBTREE_H_
