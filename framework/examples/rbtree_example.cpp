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

#include <iostream>

#include "cybertron/scheduler/policy/rbtree.h"

using namespace apollo::cybertron::scheduler;

class MyNode : public RBNode {
 public:
  explicit MyNode(int k) : key_(k) {}

  int key_;

  virtual bool Compare(RBNode *);
  virtual int Compare(void *);
  virtual void Print();
};

bool MyNode::Compare(RBNode *node) {
  MyNode *n = reinterpret_cast<MyNode *>(node);
  
  if (key_ > n->key_)
    return true;
  else
    return false; 
}

int MyNode::Compare(void *key) {
  if (key_ == *(int *)key)
    return 0;
  else if (key_ > *(int *)key)
    return 1;
  else
    return -1;
}

void MyNode::Print() {
  std::cout << "key:" << key_ << " color:" << (parent_color_ & 1) << "\n";
}

int main(int argc, char **argv) {
  int len = 9, i;
  RBTree t1, t2, t3; 
  MyNode *n1[len], *n2[len], *n3[len];
  int a1[] = { 10, 40, 30, 60, 90, 70, 20, 50, 80 };
  int a2[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
  int a3[] = {9, 8, 7, 6, 5, 4, 3, 2, 1};

  for (i = 0; i < 9; i++) {
    n1[i] = new MyNode(a1[i]);
    n2[i] = new MyNode(a2[i]);
    n3[i] = new MyNode(a3[i]);
  }

  for (i = 0; i < 9; i++) {
    t1.Insert(n1[i]);
    t2.Insert(n2[i]);
    t3.Insert(n3[i]);
  }

  t1.Preorder();
  std::cout << "\n";
  t2.Preorder();
  std::cout << "\n";
  t3.Preorder();
  std::cout << "\n";

  int val = 60;
  RBNode *tmp = t1.Find(&val);
  if (tmp) {
    std::cout << "node '" << val << "' is found:\n";
    tmp->Print();
    t1.Delete(tmp);
    std::cout << "Preodrer after delete node '" << val << "':\n";
    t1.Preorder();
  } else
    std::cout << "node '" << val << "' is not found.\n";

  return 0;
}

