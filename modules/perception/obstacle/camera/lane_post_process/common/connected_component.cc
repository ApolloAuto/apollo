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

#include "modules/perception/obstacle/camera/lane_post_process/common/connected_component.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace apollo {
namespace perception {

using std::atan2;
using std::max;
using std::min;
using std::numeric_limits;
using std::shared_ptr;
using std::sqrt;
using std::swap;
using std::unordered_set;
using std::vector;

const ScalarType kEpsCross = 0.001;
const ScalarType kCloseToBboxPercentage = 0.0;
const ScalarType kCloseEdgeLength = 10.0;

/** DisjointSet **/
int DisjointSet::Add() {
  int cur_size = static_cast<int>(disjoint_array_.size());
  disjoint_array_.push_back(cur_size);
  ++subset_num_;
  return cur_size;
}

int DisjointSet::Find(int x) {
  int root = x;
  while (root != disjoint_array_[root]) {
    root = disjoint_array_[root];
  }

  while (disjoint_array_[x] != root) {
    int temp = disjoint_array_[x];
    disjoint_array_[x] = root;
    x = temp;
  }

  return root;
}

void DisjointSet::Unite(int x, int y) {
  if (x == y) {
    return;
  }

  int x_root = Find(x);
  int y_root = Find(y);
  if (x_root == y_root) {
    return;
  } else if (x_root < y_root) {
    disjoint_array_[y_root] = x_root;
  } else {
    disjoint_array_[x_root] = y_root;
  }

  --subset_num_;
}

/** ConnectedComponent **/
void ConnectedComponent::AddPixel(int x, int y) {
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

void ConnectedComponent::FindBboxPixels() {
  bbox_.bbox_pixel_idx.reset(new vector<int>);
  for (int i = 0; i < pixel_count_; ++i) {
    if (pixels_->at(i).x == bbox_.x_min || pixels_->at(i).x == bbox_.x_max ||
        pixels_->at(i).y == bbox_.y_min || pixels_->at(i).y == bbox_.y_max) {
      bbox_.bbox_pixel_idx->push_back(i);
    }
  }
}

ConnectedComponent::BoundingBoxSplitType ConnectedComponent::DetermineSplit(
    ScalarType split_siz) {
  int height = bbox_.y_max - bbox_.y_min + 1;
  int width = bbox_.x_max - bbox_.x_min + 1;
  ScalarType diag_len = sqrt(static_cast<ScalarType>(height * width));
  if (diag_len >= split_siz) {
    bbox_.split = (height < 5) ? BoundingBoxSplitType::HORIZONTAL
                               : BoundingBoxSplitType::VERTICAL;
  } else {
    bbox_.split = BoundingBoxSplitType::NONE;
  }
  return bbox_.split;
}

void ConnectedComponent::FindContourForSplit() {
  if (bbox_.split == BoundingBoxSplitType::NONE) {
    return;
  }

  // initialize contours
  if (bbox_.split == BoundingBoxSplitType::VERTICAL) {
    bbox_.left_contour.reset(
        new std::vector<int>(bbox_.y_max - bbox_.y_min + 1, bbox_.x_max));
    bbox_.right_contour.reset(
        new std::vector<int>(bbox_.y_max - bbox_.y_min + 1, bbox_.x_min));
  } else if (bbox_.split == BoundingBoxSplitType::HORIZONTAL) {
    bbox_.up_contour.reset(
        new std::vector<int>(bbox_.x_max - bbox_.x_min + 1, bbox_.y_max));
    bbox_.down_contour.reset(
        new std::vector<int>(bbox_.x_max - bbox_.x_min + 1, bbox_.y_min));
  }

  // find contour pixels
  for (int i = 0; i < pixel_count_; ++i) {
    // get contour pixels if need splitting
    if (bbox_.split == BoundingBoxSplitType::VERTICAL) {
      int y = pixels_->at(i).y - bbox_.y_min;
      bbox_.left_contour->at(y) =
          min(bbox_.left_contour->at(y), pixels_->at(i).x);
      bbox_.right_contour->at(y) =
          max(bbox_.right_contour->at(y), pixels_->at(i).x);
    } else if (bbox_.split == BoundingBoxSplitType::HORIZONTAL) {
      int x = pixels_->at(i).x - bbox_.x_min;
      bbox_.up_contour->at(x) = min(bbox_.up_contour->at(x), pixels_->at(i).y);
      bbox_.down_contour->at(x) =
          max(bbox_.down_contour->at(x), pixels_->at(i).y);
    }
  }
}

void ConnectedComponent::FindVertices() {
  unordered_set<int> left_boundary;
  unordered_set<int> up_boundary;
  unordered_set<int> right_boundary;
  unordered_set<int> down_boundary;
  for (auto it = bbox_.bbox_pixel_idx->begin();
       it != bbox_.bbox_pixel_idx->end(); ++it) {
    const cv::Point2i* p = &(pixels_->at(*it));
    if (p->x == x_min()) {
      left_boundary.insert(p->y);
    }
    if (p->y == y_min()) {
      up_boundary.insert(p->x);
    }
    if (p->x == x_max()) {
      right_boundary.insert(p->y);
    }
    if (p->y == y_max()) {
      down_boundary.insert(p->x);
    }
  }

  vertices_.reset(new vector<Vertex>);
  for (auto it = bbox_.bbox_pixel_idx->begin();
       it != bbox_.bbox_pixel_idx->end(); ++it) {
    const cv::Point2i* p = &(pixels_->at(*it));
    if (p->x == x_min() && p->y == y_max()) {
      // bottom-left corner
      if (down_boundary.find(p->x + 1) == down_boundary.end() ||
          left_boundary.find(p->y - 1) == left_boundary.end()) {
        vertices_->push_back(Vertex(p->x, p->y));
      }
    } else if (p->x == x_min() && p->y == y_min()) {
      // upper-left corner
      if (up_boundary.find(p->x + 1) == up_boundary.end() ||
          left_boundary.find(p->y + 1) == left_boundary.end()) {
        vertices_->push_back(Vertex(p->x, p->y));
      }
    } else if (p->x == x_max() && p->y == y_min()) {
      // upper-right corner
      if (up_boundary.find(p->x - 1) == up_boundary.end() ||
          right_boundary.find(p->y + 1) == right_boundary.end()) {
        vertices_->push_back(Vertex(p->x, p->y));
      }
    } else if (p->x == x_min() && p->y == y_max()) {
      // bottom-right corner
      if (down_boundary.find(p->x - 1) == down_boundary.end() ||
          right_boundary.find(p->y - 1) == right_boundary.end()) {
        vertices_->push_back(Vertex(p->x, p->y));
      }
    } else {
      // other bounding box pixels
      if (p->x == x_min()) {
        // on left boundary
        if (left_boundary.find(p->y - 1) == left_boundary.end() ||
            left_boundary.find(p->y + 1) == left_boundary.end()) {
          vertices_->push_back(Vertex(p->x, p->y));
        }
      } else if (p->y == y_min()) {
        // on upper boundary
        if (up_boundary.find(p->x - 1) == up_boundary.end() ||
            up_boundary.find(p->x + 1) == up_boundary.end()) {
          vertices_->push_back(Vertex(p->x, p->y));
        }
      } else if (p->x == x_max()) {
        // on right boundary
        if (right_boundary.find(p->y - 1) == right_boundary.end() ||
            right_boundary.find(p->y + 1) == right_boundary.end()) {
          vertices_->push_back(Vertex(p->x, p->y));
        }
      } else if (p->y == y_max()) {
        // on lower boundary
        if (down_boundary.find(p->x - 1) == down_boundary.end() ||
            down_boundary.find(p->x + 1) == down_boundary.end()) {
          vertices_->push_back(Vertex(p->x, p->y));
        }
      } else {
        std::cerr << "the point "
                  << "(" << p->x << ", " << p->y << ")"
                  << " is not on bounding box." << std::endl;
      }
    }
  }
}

ConnectedComponent::Edge ConnectedComponent::MakeEdge(int i, int j) {
  ConnectedComponent::Edge edge;
  edge.start_vertex_id = i;
  edge.end_vertex_id = j;
  const Vertex& start_vertex = vertices_->at(i);
  const Vertex& end_vertex = vertices_->at(j);
  edge.vec(0) = end_vertex(0) - start_vertex(0);
  edge.vec(1) = end_vertex(1) - start_vertex(1);
  edge.len = edge.vec.norm();
  edge.orie = atan2(edge.vec(1), edge.vec(0));
  return edge;
}

void ConnectedComponent::FindEdges() {
  if (GetVertexCount() <= 1) {
    return;
  }

  // construct candidate edges from vertices
  edges_.reset(new vector<Edge>);
  ScalarType max_len = numeric_limits<ScalarType>::min();
  max_len_edge_id_ = -1;
  for (int i = 0; i < GetVertexCount(); ++i) {
    for (int j = i + 1; j < GetVertexCount(); ++j) {
      if (vertices_->at(i)(1) >= vertices_->at(j)(1)) {
        edges_->push_back(MakeEdge(i, j));
      } else {
        edges_->push_back(MakeEdge(j, i));
      }
      if (edges_->back().len > max_len) {
        max_len = edges_->back().len;
        max_len_edge_id_ = static_cast<int>(edges_->size()) - 1;
      }
    }
  }

  // initialize clockwise and anticlockwise edges
  const Edge& max_len_edge = edges_->at(max_len_edge_id_);

  clockwise_edge_->start_vertex_id = max_len_edge.start_vertex_id;
  clockwise_edge_->end_vertex_id = max_len_edge.end_vertex_id;
  clockwise_edge_->len = max_len_edge.len;
  clockwise_edge_->orie = max_len_edge.orie;
  clockwise_edge_->vec = max_len_edge.vec;

  anticlockwise_edge_->start_vertex_id = max_len_edge.start_vertex_id;
  anticlockwise_edge_->end_vertex_id = max_len_edge.end_vertex_id;
  anticlockwise_edge_->len = max_len_edge.len;
  anticlockwise_edge_->orie = max_len_edge.orie;
  anticlockwise_edge_->vec = max_len_edge.vec;

  // find the clockwise and anticlockwise edges
  Vertex q;
  Displacement new_vec;
  ScalarType cross;
  for (int i = 0; i < GetVertexCount(); ++i) {
    const Vertex& p = vertices_->at(i);

    // search the clockwise edge
    q = vertices_->at(clockwise_edge_->start_vertex_id);
    new_vec = p - q;
    cross = clockwise_edge_->vec(0) * new_vec(1) -
            new_vec(0) * clockwise_edge_->vec(1);
    if (cross > kEpsCross) {
      if (clockwise_edge_->vec(0) >= 0) {
        // from left to right
        if (p(0) == static_cast<ScalarType>(x_max()) ||
            p(1) == static_cast<ScalarType>(y_min())) {
          clockwise_edge_->end_vertex_id = i;
        } else {
          clockwise_edge_->start_vertex_id = i;
        }
      } else {
        // from right to left
        if (p(0) == static_cast<ScalarType>(x_min()) ||
            p(1) == static_cast<ScalarType>(y_min())) {
          clockwise_edge_->end_vertex_id = i;
        } else {
          clockwise_edge_->start_vertex_id = i;
        }
      }
      const Vertex& new_start_vertex =
          vertices_->at(clockwise_edge_->start_vertex_id);
      const Vertex& new_end_vertex =
          vertices_->at(clockwise_edge_->end_vertex_id);
      clockwise_edge_->vec(0) = new_end_vertex(0) - new_start_vertex(0);
      clockwise_edge_->vec(1) = new_end_vertex(1) - new_start_vertex(1);
    }

    // search the anticlockwise edge
    q = vertices_->at(anticlockwise_edge_->start_vertex_id);
    new_vec = p - q;
    cross = anticlockwise_edge_->vec(0) * new_vec(1) -
            new_vec(0) * anticlockwise_edge_->vec(1);
    if (cross < -kEpsCross) {
      if (anticlockwise_edge_->vec(0) <= 0) {
        // from right to left
        if (p(0) == static_cast<ScalarType>(x_min()) ||
            p(1) == static_cast<ScalarType>(y_min())) {
          anticlockwise_edge_->end_vertex_id = i;
        } else {
          anticlockwise_edge_->start_vertex_id = i;
        }
      } else {
        // from left to right
        if (p(0) == static_cast<ScalarType>(x_max()) ||
            p(1) == static_cast<ScalarType>(y_min())) {
          anticlockwise_edge_->end_vertex_id = i;
        } else {
          anticlockwise_edge_->start_vertex_id = i;
        }
      }
      const Vertex& new_start_vertex =
          vertices_->at(anticlockwise_edge_->start_vertex_id);
      const Vertex& new_end_vertex =
          vertices_->at(anticlockwise_edge_->end_vertex_id);
      anticlockwise_edge_->vec(0) = new_end_vertex(0) - new_start_vertex(0);
      anticlockwise_edge_->vec(1) = new_end_vertex(1) - new_start_vertex(1);
    }
  }

  clockwise_edge_->len = clockwise_edge_->vec.norm();
  clockwise_edge_->orie =
      atan2(clockwise_edge_->vec(1), clockwise_edge_->vec(0));

  anticlockwise_edge_->len = anticlockwise_edge_->vec.norm();
  anticlockwise_edge_->orie =
      atan2(anticlockwise_edge_->vec(1), anticlockwise_edge_->vec(0));

  clockwise_edges_->push_back(Edge());
  clockwise_edges_->back().start_vertex_id = clockwise_edge_->start_vertex_id;
  clockwise_edges_->back().end_vertex_id = clockwise_edge_->end_vertex_id;
  clockwise_edges_->back().vec = clockwise_edge_->vec;
  clockwise_edges_->back().orie = clockwise_edge_->orie;
  clockwise_edges_->back().len = clockwise_edge_->len;

  anticlockwise_edges_->push_back(Edge());
  anticlockwise_edges_->back().start_vertex_id =
      anticlockwise_edge_->start_vertex_id;
  anticlockwise_edges_->back().end_vertex_id =
      anticlockwise_edge_->end_vertex_id;
  anticlockwise_edges_->back().vec = anticlockwise_edge_->vec;
  anticlockwise_edges_->back().orie = anticlockwise_edge_->orie;
  anticlockwise_edges_->back().len = anticlockwise_edge_->len;

  if (max_len_edge.vec(0) >= 0) {
    // direction from left to right
    inner_edge_ = clockwise_edge_;
    inner_edges_ = clockwise_edges_;
  } else {
    // direction from right to left
    inner_edge_ = anticlockwise_edge_;
    inner_edges_ = anticlockwise_edges_;
  }
}

void ConnectedComponent::SplitContourVertical(int start_vertex_id,
                                              int end_vertex_id, int len_split,
                                              bool is_clockwise) {
  int start_pos =
      static_cast<int>(vertices_->at(start_vertex_id)(1));          // y_start
  int end_pos = static_cast<int>(vertices_->at(end_vertex_id)(1));  // y_end

  int height = start_pos - end_pos + 1;
  vector<int> lens = GetSplitRanges(height, len_split);
  for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
    end_pos = start_pos - lens[k] + 1;
    int x = is_clockwise ? bbox_.right_contour->at(end_pos - this->y_min())
                         : bbox_.left_contour->at(end_pos - this->y_min());
    vertices_->push_back(Vertex(x, end_pos));
    (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
        ->push_back(
            MakeEdge(start_vertex_id, static_cast<int>(vertices_->size()) - 1));
    start_pos = end_pos - 1;
    x = is_clockwise ? bbox_.right_contour->at(start_pos - this->y_min())
                     : bbox_.left_contour->at(start_pos - this->y_min());
    vertices_->push_back(Vertex(x, start_pos));
    start_vertex_id = static_cast<int>(vertices_->size()) - 1;
  }
  (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
      ->push_back(MakeEdge(start_vertex_id, end_vertex_id));
}

void ConnectedComponent::SplitContourVertical(int len_split, bool is_clockwise,
                                              int start_pos, int end_pos) {
  int height = start_pos - end_pos + 1;
  vector<int> lens = GetSplitRanges(height, len_split);

  // create start and end vertice
  int x = is_clockwise ? bbox_.right_contour->at(start_pos - this->y_min())
                       : bbox_.left_contour->at(start_pos - this->y_min());
  vertices_->push_back(Vertex(x, start_pos));
  int start_vertex_id = static_cast<int>(vertices_->size()) - 1;

  x = is_clockwise ? bbox_.right_contour->at(end_pos - this->y_min())
                   : bbox_.left_contour->at(end_pos - this->y_min());
  vertices_->push_back(Vertex(x, end_pos));
  int end_vertex_id = static_cast<int>(vertices_->size()) - 1;

  for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
    end_pos = start_pos - lens[k] + 1;
    x = is_clockwise ? bbox_.right_contour->at(end_pos - this->y_min())
                     : bbox_.left_contour->at(end_pos - this->y_min());
    vertices_->push_back(Vertex(x, end_pos));
    (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
        ->push_back(
            MakeEdge(start_vertex_id, static_cast<int>(vertices_->size()) - 1));
    start_pos = end_pos - 1;
    x = is_clockwise ? bbox_.right_contour->at(start_pos - this->y_min())
                     : bbox_.left_contour->at(start_pos - this->y_min());
    vertices_->push_back(Vertex(x, start_pos));
    start_vertex_id = static_cast<int>(vertices_->size()) - 1;
  }
  (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
      ->push_back(MakeEdge(start_vertex_id, end_vertex_id));
}

void ConnectedComponent::SplitContourHorizontal(int start_vertex_id,
                                                int end_vertex_id,
                                                int len_split,
                                                bool is_clockwise) {
  int start_pos = static_cast<int>(vertices_->at(start_vertex_id)(0));
  int end_pos = static_cast<int>(vertices_->at(end_vertex_id)(0));
  if (start_pos <= end_pos) {
    // direction from left to right
    int width = end_pos - start_pos + 1;
    vector<int> lens = GetSplitRanges(width, len_split);
    for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
      end_pos = start_pos + lens[k] - 1;
      int y = (is_clockwise ? bbox_.down_contour : bbox_.up_contour)
                  ->at(end_pos - this->x_min());
      vertices_->push_back(Vertex(end_pos, y));
      (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
          ->push_back(MakeEdge(start_vertex_id,
                               static_cast<int>(vertices_->size()) - 1));
      start_pos = end_pos + 1;
      y = (is_clockwise ? bbox_.down_contour : bbox_.up_contour)
              ->at(start_pos - this->x_min());
      vertices_->push_back(Vertex(start_pos, y));
      start_vertex_id = static_cast<int>(vertices_->size()) - 1;
    }
    (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
        ->push_back(MakeEdge(start_vertex_id, end_vertex_id));
  } else {
    // direction from right to left
    int width = start_pos - end_pos + 1;
    vector<int> lens = GetSplitRanges(width, len_split);
    for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
      end_pos = start_pos - lens[k] + 1;
      int y = (is_clockwise ? bbox_.up_contour : bbox_.down_contour)
                  ->at(end_pos - this->x_min());
      vertices_->push_back(Vertex(end_pos, y));
      (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
          ->push_back(MakeEdge(start_vertex_id,
                               static_cast<int>(vertices_->size()) - 1));
      start_pos = end_pos - 1;
      y = (is_clockwise ? bbox_.up_contour : bbox_.down_contour)
              ->at(start_pos - this->x_min());
      vertices_->push_back(Vertex(start_pos, y));
      start_vertex_id = static_cast<int>(vertices_->size()) - 1;
    }
    (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
        ->push_back(MakeEdge(start_vertex_id, end_vertex_id));
  }
}

void ConnectedComponent::SplitContourHorizontal(int len_split,
                                                bool is_clockwise,
                                                int start_pos, int end_pos) {
  if (start_pos <= end_pos) {
    // direction from left to right
    int y = (is_clockwise ? bbox_.down_contour : bbox_.up_contour)
                ->at(start_pos - this->x_min());
    vertices_->push_back(Vertex(start_pos, y));
    int start_vertex_id = static_cast<int>(vertices_->size()) - 1;

    y = (is_clockwise ? bbox_.down_contour : bbox_.up_contour)
            ->at(end_pos - this->x_min());
    vertices_->push_back(Vertex(end_pos, y));
    int end_vertex_id = static_cast<int>(vertices_->size()) - 1;

    int width = end_pos - start_pos + 1;
    vector<int> lens = GetSplitRanges(width, len_split);
    for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
      end_pos = start_pos + lens[k] - 1;
      y = (is_clockwise ? bbox_.down_contour : bbox_.up_contour)
              ->at(end_pos - this->x_min());
      vertices_->push_back(Vertex(end_pos, y));
      (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
          ->push_back(MakeEdge(start_vertex_id,
                               static_cast<int>(vertices_->size()) - 1));
      start_pos = end_pos + 1;
      y = (is_clockwise ? bbox_.down_contour : bbox_.up_contour)
              ->at(start_pos - this->x_min());
      vertices_->push_back(Vertex(start_pos, y));
      start_vertex_id = static_cast<int>(vertices_->size()) - 1;
    }
    (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
        ->push_back(MakeEdge(start_vertex_id, end_vertex_id));
  } else {
    // direction from right to left
    int y = (is_clockwise ? bbox_.up_contour : bbox_.down_contour)
                ->at(start_pos - this->x_min());
    vertices_->push_back(Vertex(start_pos, y));
    int start_vertex_id = static_cast<int>(vertices_->size()) - 1;

    y = (is_clockwise ? bbox_.up_contour : bbox_.down_contour)
            ->at(end_pos - this->x_min());
    vertices_->push_back(Vertex(end_pos, y));
    int end_vertex_id = static_cast<int>(vertices_->size()) - 1;

    int width = start_pos - end_pos + 1;
    vector<int> lens = GetSplitRanges(width, len_split);
    for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
      end_pos = start_pos - lens[k] + 1;
      y = (is_clockwise ? bbox_.up_contour : bbox_.down_contour)
              ->at(end_pos - this->x_min());
      vertices_->push_back(Vertex(end_pos, y));
      (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
          ->push_back(MakeEdge(start_vertex_id,
                               static_cast<int>(vertices_->size()) - 1));
      start_pos = end_pos - 1;
      y = (is_clockwise ? bbox_.up_contour : bbox_.down_contour)
              ->at(start_pos - this->x_min());
      vertices_->push_back(Vertex(start_pos, y));
      start_vertex_id = static_cast<int>(vertices_->size()) - 1;
    }
    (is_clockwise ? clockwise_edges_ : anticlockwise_edges_)
        ->push_back(MakeEdge(start_vertex_id, end_vertex_id));
  }
}

void ConnectedComponent::SplitContour(int split_len) {
  if (bbox_.split == BoundingBoxSplitType::NONE) {
    return;
  }

  clockwise_edges_->clear();
  anticlockwise_edges_->clear();

  if (bbox_.split == BoundingBoxSplitType::VERTICAL) {
    // split clockwise contour
    SplitContourVertical(split_len, true, y_max(), y_min());
    // split anticlockwise contour
    SplitContourVertical(split_len, false, y_max(), y_min());

  } else if (bbox_.split == BoundingBoxSplitType::HORIZONTAL) {
    // split clockwise contour
    if (vertices_->at(clockwise_edge_->start_vertex_id)(0) <=
        vertices_->at(clockwise_edge_->end_vertex_id)(0)) {
      SplitContourHorizontal(split_len, true, x_min(), x_max());
    } else {
      SplitContourHorizontal(split_len, true, x_max(), x_min());
    }
    // split anticlockwise contour
    if (vertices_->at(anticlockwise_edge_->start_vertex_id)(0) <=
        vertices_->at(anticlockwise_edge_->end_vertex_id)(0)) {
      SplitContourHorizontal(split_len, false, x_min(), x_max());
    } else {
      SplitContourHorizontal(split_len, false, x_max(), x_min());
    }

  } else {
    std::cerr << "unknown bounding box split type: "
              << bbox_.split << std::endl;
  }
}

void ConnectedComponent::Process(ScalarType split_siz, int split_len) {
  FindBboxPixels();
  FindVertices();
  FindEdges();
  if (DetermineSplit(split_siz) !=
      ConnectedComponent::BoundingBoxSplitType::NONE) {
    FindContourForSplit();
    SplitContour(split_len);
  }
}

vector<int> ConnectedComponent::GetSplitRanges(int siz, int len_split) {
  if (siz <= 0) {
    std::cerr << "siz should be a positive number: "
              << siz << std::endl;
  }
  if (len_split <= 0) {
    std::cerr << "len_split should be a positive number: "
              << len_split << std::endl;
  }

  int num_split = siz / len_split;
  int remainder = siz % len_split;
  vector<int> lens(num_split, len_split);

  if (lens.size() == 0 || remainder > len_split / 2) {
    lens.push_back(remainder);
    ++num_split;
  } else {
    lens.back() += remainder;
  }

  return lens;
}

/** connected component generator **/
ConnectedComponentGenerator::ConnectedComponentGenerator(int image_width,
                                                         int image_height)
    : image_width_(image_width),
      image_height_(image_height),
      width_(image_width),
      height_(image_height),
      roi_x_min_(0),
      roi_y_min_(0),
      roi_x_max_(image_width - 1),
      roi_y_max_(image_height - 1) {
  total_pix_ =
      static_cast<size_t>(image_width_) * static_cast<size_t>(image_height_);
  labels_.Init(total_pix_);
  frame_label_.resize(total_pix_, -1);
  root_map_.reserve(total_pix_);
}

ConnectedComponentGenerator::ConnectedComponentGenerator(int image_width,
                                                         int image_height,
                                                         cv::Rect roi)
    : image_width_(image_width),
      image_height_(image_height),
      width_(roi.width),
      height_(roi.height),
      roi_x_min_(roi.x),
      roi_y_min_(roi.y),
      roi_x_max_(roi.x + roi.width - 1),
      roi_y_max_(roi.y + roi.height - 1) {
  if (roi_x_min_ < 0) {
    std::cerr << "x_min is less than zero: " << roi_x_min_ << std::endl;
  }
  if (roi_y_min_ < 0) {
    std::cerr << "y_min is less than zero: " << roi_y_min_ << std::endl;
  }
  if (roi_x_max_ >= image_width_) {
    std::cerr << "x_max is larger than image width: " << roi_x_max_ << "|"
              << image_width_ << std::endl;
  }
  if (roi_y_max_ >= image_height_) {
    std::cerr << "y_max is larger than image height: " << roi_y_max_ << "|"
              << image_height_ << std::endl;
  }
  total_pix_ = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  labels_.Init(total_pix_);
  frame_label_.resize(total_pix_, -1);
  root_map_.reserve(total_pix_);
}

bool ConnectedComponentGenerator::FindConnectedComponents(
    const cv::Mat& lane_map, vector<shared_ptr<ConnectedComponent>>* cc) {
  if (lane_map.empty()) {
    std::cerr << "input lane map is empty" << std::endl;
    return false;
  }
  if (lane_map.type() != CV_8UC1) {
    std::cerr << "input lane map type is not CV_8UC1" << std::endl;
    return false;
  }

  if (lane_map.cols != image_width_) {
    std::cerr << "The width of input lane map does not match" << std::endl;
    return false;
  }
  if (lane_map.rows != image_height_) {
    std::cerr << "The height of input lane map does not match" << std::endl;
    return false;
  }

  if (cc == NULL) {
    std::cerr << "the pointer of output connected components is null."
              << std::endl;
    return false;
  }

  cc->clear();

  labels_.Reset();
  root_map_.clear();

  int x = 0;
  int y = 0;
  const uchar* cur_p = NULL;
  const uchar* prev_p = lane_map.ptr<uchar>(roi_y_min_);
  int left_val = 0;
  int up_val = 0;
  int cur_idx = 0;
  int left_idx = 0;
  int up_idx = 0;

  // first loop logic
  for (y = roi_y_min_; y <= roi_y_max_; y++) {
    cur_p = lane_map.ptr<uchar>(y);
    for (x = roi_x_min_; x <= roi_x_max_; x++, cur_idx++) {
      left_idx = cur_idx - 1;
      up_idx = cur_idx - image_width_;

      if (x == roi_x_min_) {
        left_val = 0;
      } else {
        left_val = cur_p[x - 1];
      }

      if (y == roi_y_min_) {
        up_val = 0;
      } else {
        up_val = prev_p[x];
      }

      if (cur_p[x] > 0) {
        if (left_val == 0 && up_val == 0) {
          // current pixel is foreground and has no connected neighbors
          frame_label_[cur_idx] = labels_.Add();
          root_map_.push_back(-1);
        } else if (left_val != 0 && up_val == 0) {
          // current pixel is foreground and has left neighbor connected
          frame_label_[cur_idx] = frame_label_[left_idx];
        } else if (left_val == 0 && up_val != 0) {
          // current pixel is foreground and has up neighbor connect
          frame_label_[cur_idx] = frame_label_[up_idx];
        } else {
          // current pixel is foreground and is connected to left and up
          // neighbors
          frame_label_[cur_idx] =
              (frame_label_[left_idx] > frame_label_[up_idx])
                  ? frame_label_[up_idx]
                  : frame_label_[left_idx];
          labels_.Unite(frame_label_[left_idx], frame_label_[up_idx]);
        }
      } else {
        frame_label_[cur_idx] = -1;
      }
    }  // end for x
    prev_p = cur_p;
  }  // end for y
  if (root_map_.size() != labels_.Num()) {
    std::cerr << "the size of root map and labels are not equal." << std::endl;
    return false;
  }
  std::cout << "subset number = " << labels_.Size() << std::endl;

  // second loop logic
  cur_idx = 0;
  int curt_label = 0;
  int cc_count = 0;
  for (y = roi_y_min_; y <= roi_y_max_; y++) {
    for (x = roi_x_min_; x <= roi_x_max_; x++, cur_idx++) {
      curt_label = frame_label_[cur_idx];
      if (curt_label >= 0) {
        if (curt_label >= static_cast<int>(labels_.Num())) {
          std::cerr << "curt_label should be smaller than labels.num(): "
                    << curt_label << " vs. " << labels_.Num() << std::endl;
          return false;
        }
        curt_label = labels_.Find(curt_label);
        if (curt_label >= static_cast<int>(root_map_.size())) {
          std::cerr << "curt_label should be smaller than root_map.size() "
                    << curt_label << " vs. " << root_map_.size() << std::endl;
          return false;
        }

        if (root_map_[curt_label] != -1) {
          cc->at(root_map_[curt_label])->AddPixel(x, y);
        } else {
          cc->push_back(std::make_shared<ConnectedComponent>(x, y));
          root_map_[curt_label] = cc_count++;
        }
      }
    }  // end for x
  }    // end for y
  std::cout << "The number of cc = " << cc_count << std::endl;

  return true;
}

}  // namespace perception
}  // namespace apollo
