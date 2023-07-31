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
#pragma once

#include <vector>

#include "modules/perception/common/algorithm/i_lib/core/i_alloc.h"
#include "modules/perception/common/lib/thread/thread_worker.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_label_image.h"

namespace apollo {
namespace perception {
namespace lidar {

class SppCCDetector {
 public:
  SppCCDetector() = default;

  ~SppCCDetector() {
    if (nodes_ != nullptr) {
      algorithm::IFree2(&nodes_);
    }
  }
  // @brief: initialize detector
  // @param [in]: rows of feature map
  // @param [in]: cols of feature map
  void Init(int rows, int cols) {
    if (rows_ * cols_ != rows * cols) {
      if (nodes_ != nullptr) {
        algorithm::IFree2(&nodes_);
      }
      nodes_ = algorithm::IAlloc2<Node>(rows, cols);
      rows_ = static_cast<int>(rows);
      cols_ = static_cast<int>(cols);
    }
    CleanNodes();
  }
  // @brief: set data for clusterin
  // @param [in]: probability map
  // @param [in]: center offset map
  // @param [in]: scale of offset map
  // @param [in]: objectness threshold
  void SetData(const float* const* prob_map, const float* offset_map,
               float scale, float objectness_threshold);
  // @brief: detect clusters
  // @param [out]: label image
  // @return: label number
  size_t Detect(SppLabelImage* labels);

 private:
  // @brief: build node matrix given start row index and end row index
  // @param [in]: start row index, inclusive
  // @param [in]: end row index, exclusive
  // @param [out]: state of build nodes
  bool BuildNodes(int start_row_index, int end_row_index);
  // @brief: traverse node matrix
  void TraverseNodes();
  // @brief: union adjacent nodes
  void UnionNodes();
  // @brief: collect clusters to label map
  size_t ToLabelMap(SppLabelImage* labels);
  // @brief: clean node matrix
  bool CleanNodes();

 private:
  struct Node {
    uint32_t center_node = 0;
    uint32_t parent = 0;
    uint16_t id = 0;
    // Note, we compress node_rank, traversed, is_center and is_object
    // in one 16bits variable, the arrangemant is as following
    // |is_center(1bit)|is_object(1bit)|traversed(3bit)|node_rank(11bit)|
    uint16_t status = 0;

    inline uint16_t get_node_rank() { return status & 2047; }
    inline void set_node_rank(uint16_t node_rank) {
      status &= 63488;
      status |= node_rank;
    }
    inline uint16_t get_traversed() {
      uint16_t pattern = 14336;
      return static_cast<uint16_t>((status & pattern) >> 11);
    }
    inline void set_traversed(uint16_t traversed) {
      status &= 51199;
      uint16_t pattern = 7;
      status |= static_cast<uint16_t>((traversed & pattern) << 11);
    }
    inline bool is_center() { return static_cast<bool>(status & 32768); }
    inline void set_is_center(bool is_center) {
      if (is_center) {
        status |= 32768;
      } else {
        status &= 32767;
      }
    }
    inline bool is_object() { return static_cast<bool>(status & 16384); }
    inline void set_is_object(bool is_object) {
      if (is_object) {
        status |= 16384;  // 2^14
      } else {
        status &= 49151;  // 65535 - 2^14
      }
    }
  };
  // @brief: tranverse a node
  // @param [in]: input node
  void Traverse(Node* x);
  // @brief: find root of input node and compress path
  // @param [in]: input node
  // @return: root node
  Node* DisjointSetFindLoop(Node* x);
  // @brief: find root of input node
  // @param [in]: input node
  // @return: root node
  Node* DisjointSetFind(Node* x);
  // @brief: union of two sets
  // @param [in]: input two nodes
  void DisjointSetUnion(Node* x, Node* y);

 private:
  int rows_ = 0;
  int cols_ = 0;
  Node** nodes_ = nullptr;

  const float* const* prob_map_ = nullptr;
  const float* offset_map_ = nullptr;
  float scale_ = 0.f;
  float objectness_threshold_ = 0.f;

  lib::ThreadWorker worker_;
  bool first_process_ = true;

 private:
  static const size_t kDefaultReserveSize = 500;
};  // class SppCCDetector

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
