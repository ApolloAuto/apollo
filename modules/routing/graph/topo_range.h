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

#ifndef BAIDU_ADU_ROUTING_GRAPH_TOPO_RANGE_H
#define BAIDU_ADU_ROUTING_GRAPH_TOPO_RANGE_H

#include <vector>
#include <algorithm>
#include <queue>
#include "glog/logging.h"

namespace adu {
namespace routing {

class TopoNode;

class NodeSRange {
public:
    static bool is_enough_for_change_lane(double start_s, double end_s);
    static bool is_enough_for_change_lane(double length);

public:
    NodeSRange();
    NodeSRange(double s1, double s2);
    NodeSRange(const NodeSRange& other);
    ~NodeSRange() = default;

    bool operator < (const NodeSRange& other) const;
    bool is_valid() const;
    double start_s() const;
    double end_s() const;
    bool is_enough_for_change_lane() const;
    double length() const;

    void set_start_s(double start_s);
    void set_end_s(double end_s);
    void set_range_s(double start_s, double end_s);
    bool merge_range_overlap(const NodeSRange& other);

private:
    double _start_s;
    double _end_s;
};

class NodeWithRange : public NodeSRange {
public:
    NodeWithRange(const NodeSRange& range, const TopoNode* node);
    ~NodeWithRange();
    bool operator < (const NodeWithRange& other) const;

    const TopoNode* topo_node() const;

private:
    const TopoNode* _topo_node;
};

template <typename T>
int binary_search_for_s_larger(const std::vector<T>& sorted_vec,
                               double value_s) {
    if (sorted_vec.empty()) {
        return -1;
    }
    int start_index = 0;
    int end_index = sorted_vec.size() - 1;
    int internal_s = 0.0;
    int middle_index = 0;
    while (end_index - start_index > 1) {
        middle_index = (start_index + end_index) / 2;
        internal_s = sorted_vec[middle_index].start_s();
        if (internal_s > value_s) {
            end_index = middle_index;
        } else {
            start_index = middle_index;
        }
    }
    double end_s = sorted_vec[start_index].end_s();
    if (value_s <= end_s) {
        return start_index;
    }
    return end_index;
}

template <typename T>
int binary_search_for_s_smaller(const std::vector<T>& sorted_vec,
                                double value_s) {
    if (sorted_vec.empty()) {
        return -1;
    }
    int start_index = 0;
    int end_index = sorted_vec.size() - 1;
    int internal_s = 0.0;
    int middle_index = 0;
    while (end_index - start_index > 1) {
        middle_index = (start_index + end_index) / 2;
        internal_s = sorted_vec[middle_index].end_s();
        if (internal_s > value_s) {
            end_index = middle_index;
        } else {
            start_index = middle_index;
        }
    }
    double start_s = sorted_vec[end_index].start_s();
    if (value_s > start_s) {
        return end_index;
    }
    return start_index;
}

template <typename T>
int binary_search_check_valid_s_index(const std::vector<T>& sorted_vec,
                                      int index,
                                      double value_s) {
    if (index == -1) {
        return -1;
    }
    double start_s = sorted_vec[index].start_s();
    double end_s = sorted_vec[index].end_s();
    if (start_s <= value_s && end_s >= value_s) {
        return index;
    }
    return -1;
}

template <typename T>
int binary_search_for_start_s(const std::vector<T>& sorted_vec,
                              double value_s) {
    int index = binary_search_for_s_larger(sorted_vec, value_s);
    return binary_search_check_valid_s_index(sorted_vec, index, value_s);
}

template <typename T>
int binary_search_for_end_s(const std::vector<T>& sorted_vec,
                            double value_s) {
    int index = binary_search_for_s_smaller(sorted_vec, value_s);
    return binary_search_check_valid_s_index(sorted_vec, index, value_s);
}


} // namespace routing
} // namespace adu

#endif // BAIDU_ADU_ROUTING_GRAPH_TOPO_RANGE_H

