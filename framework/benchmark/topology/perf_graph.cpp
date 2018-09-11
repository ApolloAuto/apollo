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

#include <cmath>
#include <iostream>
#include <string>

#include "cybertron/time/time.h"
#include "cybertron/topology/container/graph.h"

using apollo::cybertron::Time;
using apollo::cybertron::topology::Edge;
using apollo::cybertron::topology::Graph;
using apollo::cybertron::topology::Vertice;

void Usage() {
  std::cout << "Usage:" << std::endl;
  std::cout << "    argv[0] program name" << std::endl;
  std::cout << "    argv[1] depth of full binary tree" << std::endl;
}

void Run(int depth) {
  int total_node_num = pow(2, depth) - 1;
  int degree_larger_than_zero_node_num = pow(2, depth - 1) - 1;
  std::cout << "total_node_num: " << total_node_num << std::endl;
  std::cout << "node_num(degree>0): " << degree_larger_than_zero_node_num
            << std::endl;
  std::cout << "***** run statistics(unit: us) *****" << std::endl;

  Graph g;
  uint64_t start = Time::Now().ToNanosecond();
  for (int i = 0; i < degree_larger_than_zero_node_num; ++i) {
    Vertice parent(std::to_string(i));
    Vertice left_child(std::to_string(2 * i + 1));
    Vertice right_child(std::to_string(2 * i + 2));
    Edge e;
    e.set_src(parent);
    e.set_dst(left_child);
    e.set_value(parent.value() + left_child.value());
    g.Insert(e);
    e.set_dst(right_child);
    e.set_value(parent.value() + right_child.value());
    g.Insert(e);
  }
  uint64_t end = Time::Now().ToNanosecond();
  std::cout << "insert: " << (end - start) / 1000 << std::endl;

  Vertice root("0");
  Vertice lhm_leaf(std::to_string(degree_larger_than_zero_node_num));
  Vertice rhm_leaf(std::to_string(total_node_num - 1));
  start = Time::Now().ToNanosecond();
  g.GetDirectionOf(root, lhm_leaf);
  end = Time::Now().ToNanosecond();
  std::cout << "query from root to left-hand-most leaf node: "
            << (end - start) / 1000 << std::endl;

  start = Time::Now().ToNanosecond();
  g.GetDirectionOf(rhm_leaf, root);
  end = Time::Now().ToNanosecond();
  std::cout << "query from right-hand-most leaf node to root: "
            << (end - start) / 1000 << std::endl;

  Vertice second_level_left("1");
  Vertice second_level_right("2");
  start = Time::Now().ToNanosecond();
  g.GetDirectionOf(second_level_left, second_level_right);
  end = Time::Now().ToNanosecond();
  std::cout << "query from node(idx:1) to node(idx:2): " << (end - start) / 1000
            << std::endl;
}

int main(int argc, char* argv[]) {
  int depth = 10;
  if (argc == 2) {
    int tmp = 0;
    tmp = atoi(argv[1]);
    if (tmp <= 1) {
      std::cout << "please enter a integer larger than 1." << std::endl;
      return -1;
    }
    depth = tmp;
  } else if (argc > 2) {
    Usage();
    return -1;
  }

  std::cout << "depth: " << depth << std::endl;

  Run(depth);
  return 0;
}
