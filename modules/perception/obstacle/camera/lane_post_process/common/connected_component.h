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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_CC_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_CC_H_

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include <iostream>

// #include "modules/common/log.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/base_type.h"

namespace apollo {
namespace perception {

#ifndef NUM_RESERVE_VERTICES
#define NUM_RESERVE_VERTICES 4
#endif

#ifndef NUM_RESERVE_EDGES
#define NUM_RESERVE_EDGES 6
#endif

class DisjointSet {
 public:
  DisjointSet() : subset_num_(0) {}

  explicit DisjointSet(const size_t siz) : subset_num_(0) {
    disjoint_array_.reserve(siz);
  }

  ~DisjointSet() {}

  void Init(const size_t siz) {
    disjoint_array_.clear();
    disjoint_array_.reserve(siz);
    subset_num_ = 0;
  }

  void Reset() {
    disjoint_array_.clear();
    subset_num_ = 0;
  }

  // get the number of subsets (root nodes)
  int Size() const { return subset_num_; }
  // get the total number of elements
  size_t Num() const { return disjoint_array_.size(); }
  // add a new element
  int Add();
  // find the root element of x
  int Find(int x);
  // union two elements x and y
  void Unite(int x, int y);

 private:
  std::vector<int> disjoint_array_;
  int subset_num_;
};

class ConnectedComponent {
 public:
  typedef Eigen::Matrix<ScalarType, 2, 1> Vertex;
  typedef Eigen::Matrix<ScalarType, 2, 1> Displacement;

  enum BoundingBoxSplitType {
    NONE = -1,   // do not split
    VERTICAL,    // split in vertical direction (y)
    HORIZONTAL,  // split in horizontal direction (x)
  };

  struct Edge {
    int start_vertex_id;
    int end_vertex_id;
    Displacement vec;
    ScalarType len;
    ScalarType orie;

    Edge()
        : start_vertex_id(-1),
          end_vertex_id(-1),
          vec(0.0, 0.0),
          len(0.0),
          orie(0.0) {}

    int get_start_vertex_id() const { return start_vertex_id; }

    int get_end_vertex_id() const { return end_vertex_id; }
  };

  struct BoundingBox {
    int x_min;  // left
    int y_min;  // up
    int x_max;  // right
    int y_max;  // down
    std::shared_ptr<std::vector<int>> bbox_pixel_idx;
    BoundingBoxSplitType split;
    std::shared_ptr<std::vector<int>> left_contour;
    std::shared_ptr<std::vector<int>> up_contour;
    std::shared_ptr<std::vector<int>> right_contour;
    std::shared_ptr<std::vector<int>> down_contour;

    BoundingBox()
        : x_min(-1),
          y_min(-1),
          x_max(-1),
          y_max(-1),
          split(BoundingBoxSplitType::NONE) {
      bbox_pixel_idx = std::make_shared<std::vector<int>>();
      left_contour = std::make_shared<std::vector<int>>();
      up_contour = std::make_shared<std::vector<int>>();
      right_contour = std::make_shared<std::vector<int>>();
      down_contour = std::make_shared<std::vector<int>>();
    }

    BoundingBox(int x, int y)
        : x_min(x),
          y_min(y),
          x_max(x),
          y_max(y),
          split(BoundingBoxSplitType::NONE) {
      bbox_pixel_idx = std::make_shared<std::vector<int>>();
      left_contour = std::make_shared<std::vector<int>>();
      up_contour = std::make_shared<std::vector<int>>();
      right_contour = std::make_shared<std::vector<int>>();
      down_contour = std::make_shared<std::vector<int>>();
    }

    int width() const { return x_max - x_min + 1; }

    int height() const { return y_max - y_min + 1; }
  };

  ConnectedComponent() : pixel_count_(0), bbox_() {
    pixels_ = std::make_shared<std::vector<cv::Point2i>>();
    vertices_ = std::make_shared<std::vector<Vertex>>();
    vertices_->reserve(NUM_RESERVE_VERTICES);
    edges_ = std::make_shared<std::vector<Edge>>();
    edges_->reserve(NUM_RESERVE_EDGES);
    max_len_edge_id_ = -1;
    clockwise_edge_ = std::make_shared<Edge>();
    anticlockwise_edge_ = std::make_shared<Edge>();
    inner_edge_ = std::make_shared<Edge>();
    clockwise_edges_ = std::make_shared<std::vector<Edge>>();
    anticlockwise_edges_ = std::make_shared<std::vector<Edge>>();
    inner_edges_ = std::make_shared<std::vector<Edge>>();
  }

  ConnectedComponent(int x, int y) : pixel_count_(1), bbox_(x, y) {
    pixels_ = std::make_shared<std::vector<cv::Point2i>>();
    pixels_->push_back(cv::Point(x, y));
    vertices_ = std::make_shared<std::vector<Vertex>>();
    vertices_->reserve(NUM_RESERVE_VERTICES);
    edges_ = std::make_shared<std::vector<Edge>>();
    edges_->reserve(NUM_RESERVE_EDGES);
    max_len_edge_id_ = -1;
    clockwise_edge_ = std::make_shared<Edge>();
    anticlockwise_edge_ = std::make_shared<Edge>();
    inner_edge_ = std::make_shared<Edge>();
    clockwise_edges_ = std::make_shared<std::vector<Edge>>();
    anticlockwise_edges_ = std::make_shared<std::vector<Edge>>();
    inner_edges_ = std::make_shared<std::vector<Edge>>();
  }

  ~ConnectedComponent() {}

  // CC pixels
  void AddPixel(int x, int y);
  /*
  void AddPixel(int x, int y) {
    if (pixel_count_ == 0) {
      // new bounding box
      bbox_.x_min = x;  // x_min
      bbox_.y_min = y;  // y_min
      bbox_.x_max = x;  // x_max
      bbox_.y_max = y;  // y_max
    } else {
      // extend bounding box if necessary
      if (x < bbox_.x_min) {
        bbox_.x_min = x;
      }
      if (x > bbox_.x_max) {
        bbox_.x_max = x;
      }
      if (y < bbox_.y_min) {
        bbox_.y_min = y;
      }
      if (y > bbox_.y_max) {
        bbox_.y_max = y;
      }
    }

    pixels_->push_back(cv::Point(x, y));
    pixel_count_++;
  }
  */

  int GetPixelCount() const { return pixel_count_; }
  std::shared_ptr<const std::vector<cv::Point2i>> GetPixels() const {
    return pixels_;
  }

  // bounding box
  const BoundingBox* bbox() const { return &bbox_; }
  int x_min() const { return bbox_.x_min; }
  int y_min() const { return bbox_.y_min; }
  int x_max() const { return bbox_.x_max; }
  int y_max() const { return bbox_.y_max; }

  cv::Rect GetBoundingBox() const {
    return cv::Rect(bbox_.x_min, bbox_.y_min, bbox_.x_max - bbox_.x_min + 1,
                    bbox_.y_max - bbox_.y_min + 1);
  }

  int GetBoundingBoxArea() const {
    return (bbox_.x_max - bbox_.x_min + 1) * (bbox_.y_max - bbox_.y_min + 1);
  }

  // split bounding box
  BoundingBoxSplitType DetermineSplit(ScalarType split_siz);

  void FindContourForSplit();

  // bounding box pixels
  void FindBboxPixels();

  std::shared_ptr<const std::vector<int>> bbox_pixel_idx() const {
    return bbox_.bbox_pixel_idx;
  }

  int GetBboxPixelCount() const {
    return static_cast<int>(bbox_.bbox_pixel_idx->size());
  }

  // vertices
  void FindVertices();
  std::shared_ptr<const std::vector<Vertex>> GetVertices() const {
    return vertices_;
  }

  Vertex GetVertex(int vertex_id, double scale, double start_y_pos) const {
    // assert(vertex_id >= 0 && vertex_id < this->getVertexCount());
    Vertex ver_pnt = vertices_->at(vertex_id);
    ver_pnt[0] = static_cast<int>(ver_pnt[0] * scale);
    ver_pnt[1] = static_cast<int>(ver_pnt[1] * scale + start_y_pos);
    return ver_pnt;
    // return vertices_->at(vertex_id);
  }
  int GetVertexCount() const { return static_cast<int>(vertices_->size()); }

  // edges
  bool IsValidEdgeVertices(int i, int j) {
    return i >= 0 && i < this->GetVertexCount() && j >= 0 &&
           j < this->GetVertexCount() && i != j;
  }

  void FindEdges();
  int GetEdgeCount() const { return static_cast<int>(edges_->size()); }
  const Edge* GetMaxLenthEdge() const { return &edges_->at(max_len_edge_id_); }
  std::shared_ptr<const Edge> GetClockWiseEdge() const {
    return clockwise_edge_;
  }
  std::shared_ptr<const Edge> GetAntiClockWiseEdge() const {
    return anticlockwise_edge_;
  }
  std::shared_ptr<const Edge> GetInnerEdge() const { return inner_edge_; }

  void SplitContour(int split_len);
  std::shared_ptr<std::vector<Edge>> GetClockWiseEdges() const {
    return clockwise_edges_;
  }
  std::shared_ptr<std::vector<Edge>> GetAntiClockWiseEdges() const {
    return anticlockwise_edges_;
  }
  std::shared_ptr<std::vector<Edge>> GetInnerEdges() const {
    return inner_edges_;
  }

  void Process(ScalarType split_siz, int split_len);

 private:
  int Sub2Ind(int row, int col, int width) { return row * width + col; }

  void SplitContourVertical(int start_vertex_id, int end_vertex_id,
                            int len_split, bool is_clockwise);
  void SplitContourVertical(int len_split, bool is_clockwise, int start_pos,
                            int end_pos);
  void SplitContourHorizontal(int start_vertex_id, int end_vertex_id,
                              int len_split, bool is_clockwise);
  void SplitContourHorizontal(int len_split, bool is_clockwise, int start_pos,
                              int end_pos);

  std::vector<int> GetSplitRanges(int siz, int len_split);

  Edge MakeEdge(int i, int j);

  int pixel_count_;
  std::shared_ptr<std::vector<cv::Point2i>> pixels_;
  BoundingBox bbox_;
  std::shared_ptr<std::vector<Vertex>> vertices_;
  std::shared_ptr<std::vector<Edge>> edges_;
  int max_len_edge_id_;
  std::shared_ptr<Edge> clockwise_edge_, anticlockwise_edge_;
  std::shared_ptr<Edge> inner_edge_;
  std::shared_ptr<std::vector<Edge>> clockwise_edges_, anticlockwise_edges_;
  std::shared_ptr<std::vector<Edge>> inner_edges_;
};

typedef std::shared_ptr<ConnectedComponent> ConnectedComponentPtr;
typedef const std::shared_ptr<ConnectedComponent> ConnectedComponentConstPtr;

class ConnectedComponentGenerator {
 public:
  ConnectedComponentGenerator(int image_width, int image_height);
  ConnectedComponentGenerator(int image_width, int image_height, cv::Rect roi);

  bool FindConnectedComponents(
      const cv::Mat& lane_map,
      std::vector<std::shared_ptr<ConnectedComponent>>* cc);

 private:
  size_t total_pix_;
  int image_width_;
  int image_height_;

  int width_;
  int height_;
  int roi_x_min_;
  int roi_y_min_;
  int roi_x_max_;
  int roi_y_max_;

  DisjointSet labels_;
  std::vector<int> frame_label_;
  std::vector<int> root_map_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_CC_H_
