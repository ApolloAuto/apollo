// Copyright 2017 Baidu Inc. All Rights Reserved.
// @author: Zhu, Jun (zhujun08@baidu.com)
// @file: connected_components.cpp
// @brief: connected component analysis for lane detection

#include "obstacle/camera/lane_post_process/common/connected_components.hpp"
#include "util.cu"

#include <limits>
#include <utility>
#include <cmath>
#include <algorithm>

namespace adu {
namespace perception {
namespace obstacle {
namespace lane_post_process {

using std::shared_ptr;
using std::vector;
using std::unordered_set;
using std::atan2;
using std::numeric_limits;
using std::swap;
using std::max;
using std::min;
using std::sqrt;

const ScalarType kEpsCross = 0.001;
const ScalarType kCloseToBboxPercentage = 0.0;
const ScalarType kCloseEdgeLength = 10.0;

/** DisjointSet **/
// add a new element, which is a subset by itself;
int DisjointSet::add() {
    int cur_size = static_cast<int>(m_disjoint_array.size());
    m_disjoint_array.push_back(cur_size);
    ++m_subset_num;
    return cur_size;
}

int DisjointSet::find(int x) {
    if (m_disjoint_array[x] == x) {
        return x;
    }

    int y = x;
    while (y != m_disjoint_array[y]) {
        y = m_disjoint_array[y];
    }
    while (true) {
        const int z = m_disjoint_array[x];
        if (z == x) {
            break;
        }
        m_disjoint_array[x] = y;
        x = z;
    }
    return y;
}

// point the x and y to smaller root of the two
void DisjointSet::unite(int x, int y) {
    if (x == y) {
        return;
    }
    int x_root = find(x);
    int y_root = find(y);
    if (x_root == y_root) {
        return;
    } else if (x_root < y_root) {
        m_disjoint_array[y_root] = x_root;
    } else {
        m_disjoint_array[x_root] = y_root;
    }
    --m_subset_num;
}

/** ConnectedComponent **/
void ConnectedComponent::addPixel(int x, int y) {
    if (_pixel_count == 0) {
        // new bounding box
        _bbox.x_min = x;  // x_min
        _bbox.y_min = y;  // y_min
        _bbox.x_max = x;  // x_max
        _bbox.y_max = y;  // y_max
    } else {
        // extend bounding box if necessary
        if (x < _bbox.x_min) {
            _bbox.x_min = x;
        }
        if (x > _bbox.x_max) {
            _bbox.x_max = x;
        }
        if (y < _bbox.y_min) {
            _bbox.y_min = y;
        }
        if (y > _bbox.y_max) {
            _bbox.y_max = y;
        }
    }

    _pixels->push_back(cv::Point(x, y));
    _pixel_count++;
}

void ConnectedComponent::findBboxPixels() {
    _bbox.bbox_pixel_idx.reset(new std::vector<int>);
    for (int i = 0; i < _pixel_count; ++i) {
        if (_pixels->at(i).x == _bbox.x_min ||
            _pixels->at(i).x == _bbox.x_max ||
            _pixels->at(i).y == _bbox.y_min ||
            _pixels->at(i).y == _bbox.y_max) {
            _bbox.bbox_pixel_idx->push_back(i);
        }
    }
}

ConnectedComponent::BoundingBoxSplitType ConnectedComponent::determineSplit(ScalarType split_siz) {
    int height = _bbox.y_max - _bbox.y_min + 1;
    int width = _bbox.x_max - _bbox.x_min + 1;
    ScalarType diag_len = static_cast<ScalarType>(height * width);
    diag_len = sqrt(diag_len);
    if (diag_len >= split_siz) {
        _bbox.split = (height < 5) ? BoundingBoxSplitType::HORIZONTAL :
                                     BoundingBoxSplitType::VERTICAL;
    } else {
        _bbox.split = BoundingBoxSplitType::NONE;
    }
    return _bbox.split;
}

void ConnectedComponent::findContourForSplit() {
    if (_bbox.split == BoundingBoxSplitType::NONE) {
        return;
    }

    // initialize contours
    if (_bbox.split == BoundingBoxSplitType::VERTICAL) {
        _bbox.left_contour.reset(
            new std::vector<int>(_bbox.y_max - _bbox.y_min + 1, _bbox.x_max));
        _bbox.right_contour.reset(
            new std::vector<int>(_bbox.y_max - _bbox.y_min + 1, _bbox.x_min));
    } else if (_bbox.split == BoundingBoxSplitType::HORIZONTAL) {
        _bbox.up_contour.reset(
            new std::vector<int>(_bbox.x_max - _bbox.x_min + 1, _bbox.y_max));
        _bbox.down_contour.reset(
            new std::vector<int>(_bbox.x_max - _bbox.x_min + 1, _bbox.y_min));
    }

    // find contour pixels
    for (int i = 0; i < _pixel_count; ++i) {
        // get contour pixels if need splitting
        if (_bbox.split == BoundingBoxSplitType::VERTICAL) {
            int y = _pixels->at(i).y - _bbox.y_min;
            _bbox.left_contour->at(y)
                = min(_bbox.left_contour->at(y), _pixels->at(i).x);
            _bbox.right_contour->at(y)
                = max(_bbox.right_contour->at(y), _pixels->at(i).x);
        } else if (_bbox.split == BoundingBoxSplitType::HORIZONTAL) {
            int x = _pixels->at(i).x - _bbox.x_min;
            _bbox.up_contour->at(x)
                = min(_bbox.up_contour->at(x), _pixels->at(i).y);
            _bbox.down_contour->at(x)
                = max(_bbox.down_contour->at(x), _pixels->at(i).y);
        }
    }
}

void ConnectedComponent::findVertices() {
    unordered_set<int> left_boundary, up_boundary, right_boundary, down_boundary;
    for (auto it = _bbox.bbox_pixel_idx->begin();
         it != _bbox.bbox_pixel_idx->end(); ++it) {
        const cv::Point2i* p = &(_pixels->at(*it));
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

    _vertices.reset(new std::vector<Vertex>);
    for (auto it = _bbox.bbox_pixel_idx->begin();
         it != _bbox.bbox_pixel_idx->end(); ++it) {
        const cv::Point2i* p = &(_pixels->at(*it));
        if (p->x == x_min() && p->y == y_max()) {
            // bottom-left corner
            if (down_boundary.find(p->x + 1) == down_boundary.end() ||
                left_boundary.find(p->y - 1) == left_boundary.end()) {
                _vertices->push_back(Vertex(p->x, p->y));
            }
        } else if (p->x == x_min() && p->y == y_min()) {
            // upper-left corner
            if (up_boundary.find(p->x + 1) == up_boundary.end() ||
                left_boundary.find(p->y + 1) == left_boundary.end()) {
                _vertices->push_back(Vertex(p->x, p->y));
            }
        } else if (p->x == x_max() && p->y == y_min()) {
            // upper-right corner
            if (up_boundary.find(p->x - 1) == up_boundary.end() ||
                right_boundary.find(p->y + 1) == right_boundary.end()) {
                _vertices->push_back(Vertex(p->x, p->y));
            }
        } else if (p->x == x_min() && p->y == y_max()) {
            // bottom-right corner
            if (down_boundary.find(p->x - 1) == down_boundary.end() ||
                right_boundary.find(p->y - 1) == right_boundary.end()) {
                _vertices->push_back(Vertex(p->x, p->y));
            }
        } else {
            // other bounding box pixels
            if (p->x == x_min()) {
                // on left boundary
                if (left_boundary.find(p->y - 1) == left_boundary.end() ||
                    left_boundary.find(p->y + 1) == left_boundary.end()) {
                    _vertices->push_back(Vertex(p->x, p->y));
                }
            } else if (p->y == y_min()) {
                // on upper boundary
                if (up_boundary.find(p->x - 1) == up_boundary.end() ||
                    up_boundary.find(p->x + 1) == up_boundary.end()) {
                    _vertices->push_back(Vertex(p->x, p->y));
                }
            } else if (p->x == x_max()) {
                // on right boundary
                if (right_boundary.find(p->y - 1) == right_boundary.end() ||
                    right_boundary.find(p->y + 1) == right_boundary.end()) {
                    _vertices->push_back(Vertex(p->x, p->y));
                }
            } else if (p->y == y_max()) {
                // on lower boundary
                if (down_boundary.find(p->x - 1) == down_boundary.end() ||
                    down_boundary.find(p->x + 1) == down_boundary.end()) {
                    _vertices->push_back(Vertex(p->x, p->y));
                }
            } else {
                XLOG(FATAL)  << "Error: this point "
                             << "(" << p->x << ", " << p->y << ")"
                             << " is not on bounding box.";
            }
        }
    }
}

ConnectedComponent::Edge ConnectedComponent::makeEdge(int i, int j) {
    ConnectedComponent::Edge edge;
    edge.start_vertex_id = i;
    edge.end_vertex_id = j;
    const Vertex& start_vertex = _vertices->at(i);
    const Vertex& end_vertex = _vertices->at(j);
    edge.vec(0) = end_vertex(0) - start_vertex(0);
    edge.vec(1) = end_vertex(1) - start_vertex(1);
    edge.len = edge.vec.norm();
    edge.orie = atan2(edge.vec(1), edge.vec(0));
    return edge;
}

void ConnectedComponent::findEdges() {
    if (getVertexCount() <= 1) {
        return;
    }

    // construct candidate edges from vertices
    _edges.reset(new std::vector<Edge>);
    ScalarType max_len =
        numeric_limits<ScalarType>::min();
    _max_len_edge_id = -1;
    for (int i = 0; i < getVertexCount(); ++i) {
        for (int j = i + 1; j < getVertexCount(); ++j) {
            if (_vertices->at(i)(1) >= _vertices->at(j)(1)) {
                _edges->push_back(makeEdge(i, j));
            } else {
                _edges->push_back(makeEdge(j, i));
            }
            if (_edges->back().len > max_len) {
                max_len = _edges->back().len;
                _max_len_edge_id =
                    static_cast<int>(_edges->size()) - 1;
            }
        }
    }

    // initialize clockwise and anticlockwise edges
    const Edge& max_len_edge = _edges->at(_max_len_edge_id);

    _clockwise_edge->start_vertex_id = max_len_edge.start_vertex_id;
    _clockwise_edge->end_vertex_id = max_len_edge.end_vertex_id;
    _clockwise_edge->len = max_len_edge.len;
    _clockwise_edge->orie = max_len_edge.orie;
    _clockwise_edge->vec = max_len_edge.vec;

    _anticlockwise_edge->start_vertex_id = max_len_edge.start_vertex_id;
    _anticlockwise_edge->end_vertex_id = max_len_edge.end_vertex_id;
    _anticlockwise_edge->len = max_len_edge.len;
    _anticlockwise_edge->orie = max_len_edge.orie;
    _anticlockwise_edge->vec = max_len_edge.vec;

    // find the clockwise and anticlockwise edges
    Vertex q;
    Displacement new_vec;
    ScalarType cross;
    for (int i = 0; i < getVertexCount(); ++i) {
        const Vertex& p = _vertices->at(i);

        // search the clockwise edge
        q = _vertices->at(_clockwise_edge->start_vertex_id);
        new_vec = p - q;
        cross = _clockwise_edge->vec(0) * new_vec(1)
              - new_vec(0) * _clockwise_edge->vec(1);
        if (cross > kEpsCross) {
            if (_clockwise_edge->vec(0) >= 0) {
                // from left to right
                if (p(0) == static_cast<ScalarType>(x_max()) ||
                    p(1) == static_cast<ScalarType>(y_min())) {
                    _clockwise_edge->end_vertex_id = i;
                } else {
                    _clockwise_edge->start_vertex_id = i;
                }
            } else {
                // from right to left
                if (p(0) == static_cast<ScalarType>(x_min()) ||
                    p(1) == static_cast<ScalarType>(y_min())) {
                    _clockwise_edge->end_vertex_id = i;
                } else {
                    _clockwise_edge->start_vertex_id = i;
                }
            }
            const Vertex& new_start_vertex =
                _vertices->at(_clockwise_edge->start_vertex_id);
            const Vertex& new_end_vertex =
                _vertices->at(_clockwise_edge->end_vertex_id);
            _clockwise_edge->vec(0) = new_end_vertex(0) - new_start_vertex(0);
            _clockwise_edge->vec(1) = new_end_vertex(1) - new_start_vertex(1);
        }

        // search the anticlockwise edge
        q = _vertices->at(_anticlockwise_edge->start_vertex_id);
        new_vec = p - q;
        cross = _anticlockwise_edge->vec(0) * new_vec(1)
              - new_vec(0) * _anticlockwise_edge->vec(1);
        if (cross < -kEpsCross) {
            if (_anticlockwise_edge->vec(0) <= 0) {
                // from right to left
                if (p(0) == static_cast<ScalarType>(x_min()) ||
                    p(1) == static_cast<ScalarType>(y_min())) {
                    _anticlockwise_edge->end_vertex_id = i;
                } else {
                    _anticlockwise_edge->start_vertex_id = i;
                }
            } else {
                // from left to right
                if (p(0) == static_cast<ScalarType>(x_max()) ||
                    p(1) == static_cast<ScalarType>(y_min())) {
                    _anticlockwise_edge->end_vertex_id = i;
                } else {
                    _anticlockwise_edge->start_vertex_id = i;
                }
            }
            const Vertex& new_start_vertex =
                _vertices->at(_anticlockwise_edge->start_vertex_id);
            const Vertex& new_end_vertex =
                _vertices->at(_anticlockwise_edge->end_vertex_id);
            _anticlockwise_edge->vec(0) = new_end_vertex(0) - new_start_vertex(0);
            _anticlockwise_edge->vec(1) = new_end_vertex(1) - new_start_vertex(1);
        }
    }

    _clockwise_edge->len = _clockwise_edge->vec.norm();
    _clockwise_edge->orie
        = atan2(_clockwise_edge->vec(1), _clockwise_edge->vec(0));

    _anticlockwise_edge->len = _anticlockwise_edge->vec.norm();
    _anticlockwise_edge->orie
        = atan2(_anticlockwise_edge->vec(1), _anticlockwise_edge->vec(0));

    _clockwise_edges->push_back(Edge());
    _clockwise_edges->back().start_vertex_id = _clockwise_edge->start_vertex_id;
    _clockwise_edges->back().end_vertex_id = _clockwise_edge->end_vertex_id;
    _clockwise_edges->back().vec = _clockwise_edge->vec;
    _clockwise_edges->back().orie = _clockwise_edge->orie;
    _clockwise_edges->back().len = _clockwise_edge->len;

    _anticlockwise_edges->push_back(Edge());
    _anticlockwise_edges->back().start_vertex_id = _anticlockwise_edge->start_vertex_id;
    _anticlockwise_edges->back().end_vertex_id = _anticlockwise_edge->end_vertex_id;
    _anticlockwise_edges->back().vec = _anticlockwise_edge->vec;
    _anticlockwise_edges->back().orie = _anticlockwise_edge->orie;
    _anticlockwise_edges->back().len = _anticlockwise_edge->len;

    if (max_len_edge.vec(0) >= 0) {
        // direction from left to right
        _inner_edge = _clockwise_edge;
        _inner_edges = _clockwise_edges;
    } else {
        // direction from right to left
        _inner_edge = _anticlockwise_edge;
        _inner_edges = _anticlockwise_edges;
    }
}

void ConnectedComponent::splitContourVertical(int start_vertex_id, int end_vertex_id,
                                              int len_split, bool is_clockwise) {
    int start_pos = static_cast<int>(_vertices->at(start_vertex_id)(1));  // y_start
    int end_pos = static_cast<int>(_vertices->at(end_vertex_id)(1));      // y_end

    int height = start_pos - end_pos + 1;
    vector<int> lens = get_split_ranges(height, len_split);
    for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
        end_pos = start_pos - lens[k] + 1;
        int x = is_clockwise ? _bbox.right_contour->at(end_pos - this->y_min())
                             : _bbox.left_contour->at(end_pos - this->y_min());
        _vertices->push_back(Vertex(x, end_pos));
        (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
            makeEdge(start_vertex_id, static_cast<int>(_vertices->size()) - 1));
        start_pos = end_pos - 1;
        x = is_clockwise ? _bbox.right_contour->at(start_pos - this->y_min())
                         : _bbox.left_contour->at(start_pos - this->y_min());
        _vertices->push_back(Vertex(x, start_pos));
        start_vertex_id = static_cast<int>(_vertices->size()) - 1;
    }
    (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
        makeEdge(start_vertex_id, end_vertex_id));
}

void ConnectedComponent::splitContourVertical(int len_split, bool is_clockwise,
                                              int start_pos, int end_pos) {
    int height = start_pos - end_pos + 1;
    vector<int> lens = get_split_ranges(height, len_split);

    // create start and end vertice
    int x = is_clockwise ? _bbox.right_contour->at(start_pos - this->y_min())
                         : _bbox.left_contour->at(start_pos - this->y_min());
    _vertices->push_back(Vertex(x, start_pos));
    int start_vertex_id = static_cast<int>(_vertices->size()) - 1;

    x = is_clockwise ? _bbox.right_contour->at(end_pos - this->y_min())
                         : _bbox.left_contour->at(end_pos - this->y_min());
    _vertices->push_back(Vertex(x, end_pos));
    int end_vertex_id = static_cast<int>(_vertices->size()) - 1;

    for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
        end_pos = start_pos - lens[k] + 1;
        x = is_clockwise ? _bbox.right_contour->at(end_pos - this->y_min())
                         : _bbox.left_contour->at(end_pos - this->y_min());
        _vertices->push_back(Vertex(x, end_pos));
        (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
            makeEdge(start_vertex_id, static_cast<int>(_vertices->size()) - 1));
        start_pos = end_pos - 1;
        x = is_clockwise ? _bbox.right_contour->at(start_pos - this->y_min())
                         : _bbox.left_contour->at(start_pos - this->y_min());
        _vertices->push_back(Vertex(x, start_pos));
        start_vertex_id = static_cast<int>(_vertices->size()) - 1;
    }
    (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
        makeEdge(start_vertex_id, end_vertex_id));
}

void ConnectedComponent::splitContourHorizontal(int start_vertex_id, int end_vertex_id,
                                                int len_split, bool is_clockwise) {
    int start_pos = static_cast<int>(_vertices->at(start_vertex_id)(0));
    int end_pos = static_cast<int>(_vertices->at(end_vertex_id)(0));
    if (start_pos <= end_pos) {
        // direction from left to right
        int width = end_pos - start_pos + 1;
        vector<int> lens = get_split_ranges(width, len_split);
        for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
            end_pos = start_pos + lens[k] - 1;
            int y = (is_clockwise ? _bbox.down_contour : _bbox.up_contour)->at(
                end_pos - this->x_min());
            _vertices->push_back(Vertex(end_pos, y));
            (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
                makeEdge(start_vertex_id, static_cast<int>(_vertices->size()) - 1));
            start_pos = end_pos + 1;
            y = (is_clockwise ? _bbox.down_contour : _bbox.up_contour)->at(
                start_pos - this->x_min());
            _vertices->push_back(Vertex(start_pos, y));
            start_vertex_id = static_cast<int>(_vertices->size()) - 1;
        }
        (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
            makeEdge(start_vertex_id, end_vertex_id));
    } else {
        // direction from right to left
        int width = start_pos - end_pos + 1;
        vector<int> lens = get_split_ranges(width, len_split);
        for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
            end_pos = start_pos - lens[k] + 1;
            int y = (is_clockwise ? _bbox.up_contour : _bbox.down_contour)->at(
                end_pos - this->x_min());
            _vertices->push_back(Vertex(end_pos, y));
            (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
                makeEdge(start_vertex_id, static_cast<int>(_vertices->size()) - 1));
            start_pos = end_pos - 1;
            y = (is_clockwise ? _bbox.up_contour : _bbox.down_contour)->at(
                start_pos - this->x_min());
            _vertices->push_back(Vertex(start_pos, y));
            start_vertex_id = static_cast<int>(_vertices->size()) - 1;
        }
        (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
            makeEdge(start_vertex_id, end_vertex_id));
    }
}

void ConnectedComponent::splitContourHorizontal(int len_split, bool is_clockwise,
                                                int start_pos, int end_pos) {
    if (start_pos <= end_pos) {
        // direction from left to right
        int y = (is_clockwise ? _bbox.down_contour : _bbox.up_contour)->at(
            start_pos - this->x_min());
        _vertices->push_back(Vertex(start_pos, y));
        int start_vertex_id = static_cast<int>(_vertices->size()) - 1;

        y = (is_clockwise ? _bbox.down_contour : _bbox.up_contour)->at(
            end_pos - this->x_min());
        _vertices->push_back(Vertex(end_pos, y));
        int end_vertex_id = static_cast<int>(_vertices->size()) - 1;

        int width = end_pos - start_pos + 1;
        vector<int> lens = get_split_ranges(width, len_split);
        for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
            end_pos = start_pos + lens[k] - 1;
            y = (is_clockwise ? _bbox.down_contour : _bbox.up_contour)->at(
                end_pos - this->x_min());
            _vertices->push_back(Vertex(end_pos, y));
            (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
                makeEdge(start_vertex_id, static_cast<int>(_vertices->size()) - 1));
            start_pos = end_pos + 1;
            y = (is_clockwise ? _bbox.down_contour : _bbox.up_contour)->at(
                start_pos - this->x_min());
            _vertices->push_back(Vertex(start_pos, y));
            start_vertex_id = static_cast<int>(_vertices->size()) - 1;
        }
        (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
            makeEdge(start_vertex_id, end_vertex_id));
    } else {
        // direction from right to left
        int y = (is_clockwise ? _bbox.up_contour : _bbox.down_contour)->at(
            start_pos - this->x_min());
        _vertices->push_back(Vertex(start_pos, y));
        int start_vertex_id = static_cast<int>(_vertices->size()) - 1;

        y = (is_clockwise ? _bbox.up_contour : _bbox.down_contour)->at(
            end_pos - this->x_min());
        _vertices->push_back(Vertex(end_pos, y));
        int end_vertex_id = static_cast<int>(_vertices->size()) - 1;

        int width = start_pos - end_pos + 1;
        vector<int> lens = get_split_ranges(width, len_split);
        for (int k = 0; k < static_cast<int>(lens.size()) - 1; ++k) {
            end_pos = start_pos - lens[k] + 1;
            y = (is_clockwise ? _bbox.up_contour : _bbox.down_contour)->at(
                end_pos - this->x_min());
            _vertices->push_back(Vertex(end_pos, y));
            (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
                makeEdge(start_vertex_id, static_cast<int>(_vertices->size()) - 1));
            start_pos = end_pos - 1;
            y = (is_clockwise ? _bbox.up_contour : _bbox.down_contour)->at(
                start_pos - this->x_min());
            _vertices->push_back(Vertex(start_pos, y));
            start_vertex_id = static_cast<int>(_vertices->size()) - 1;
        }
        (is_clockwise ? _clockwise_edges : _anticlockwise_edges)->push_back(
            makeEdge(start_vertex_id, end_vertex_id));
    }
}

#if false  // version 1
void ConnectedComponent::splitContour(int split_len) {
    if (_bbox.split == BoundingBoxSplitType::NONE) {
        return;
    }

    _clockwise_edges->clear();
    _anticlockwise_edges->clear();

    if (_bbox.split == BoundingBoxSplitType::VERTICAL) {
        // split clockwise contour
        splitContourVertical(_clockwise_edge->start_vertex_id,
                             _clockwise_edge->end_vertex_id,
                             split_len, true);


        // split anticlockwise contour
        splitContourVertical(_anticlockwise_edge->start_vertex_id,
                             _anticlockwise_edge->end_vertex_id,
                             split_len, false);

    } else if (_bbox.split == BoundingBoxSplitType::HORIZONTAL) {
        // split clockwise contour
        splitContourHorizontal(_clockwise_edge->start_vertex_id,
                               _clockwise_edge->end_vertex_id,
                               split_len, true);
        // split anticlockwise contour
        splitContourHorizontal(_anticlockwise_edge->start_vertex_id,
                               _anticlockwise_edge->end_vertex_id,
                               split_len, false);

    } else {
        LOG(FATAL) << "unknown bounding box split type: " << _bbox.split;
    }
}
#endif

#if true  // version 2
void ConnectedComponent::splitContour(int split_len) {
    if (_bbox.split == BoundingBoxSplitType::NONE) {
        return;
    }

    _clockwise_edges->clear();
    _anticlockwise_edges->clear();

    if (_bbox.split == BoundingBoxSplitType::VERTICAL) {
        // split clockwise contour
        splitContourVertical(split_len, true, y_max(), y_min());
        // split anticlockwise contour
        splitContourVertical(split_len, false, y_max(), y_min());

    } else if (_bbox.split == BoundingBoxSplitType::HORIZONTAL) {
        // split clockwise contour
        if (_vertices->at(_clockwise_edge->start_vertex_id)(0) <=
            _vertices->at(_clockwise_edge->end_vertex_id)(0)) {
            splitContourHorizontal(split_len, true, x_min(), x_max());
        } else {
            splitContourHorizontal(split_len, true, x_max(), x_min());
        }
        // split anticlockwise contour
        if (_vertices->at(_anticlockwise_edge->start_vertex_id)(0) <=
            _vertices->at(_anticlockwise_edge->end_vertex_id)(0)) {
            splitContourHorizontal(split_len, false, x_min(), x_max());
        } else {
            splitContourHorizontal(split_len, false, x_max(), x_min());
        }

    } else {
        LOG(FATAL) << "unknown bounding box split type: " << _bbox.split;
    }
}
#endif

#if false  // version 3
void ConnectedComponent::splitContour(int split_len) {
    if (_bbox.split == BoundingBoxSplitType::NONE) {
        return;
    }

    _clockwise_edges->clear();
    _anticlockwise_edges->clear();

    ScalarType bbox_diag_len = std::sqrt(static_cast<ScalarType>(getBoundingBoxArea()));

    if (_bbox.split == BoundingBoxSplitType::VERTICAL) {
        // compute split bin size in verticle direction
        split_len = static_cast<int>(static_cast<ScalarType>(split_len) *
                                     static_cast<ScalarType>(_bbox.height()) / bbox_diag_len);
        split_len = std::max(split_len, 5);

        // split clockwise contour
        int start_pos = static_cast<int>(_vertices->at(_clockwise_edge->start_vertex_id).y());
        int end_pos = static_cast<int>(_vertices->at(_clockwise_edge->end_vertex_id).y());
        if (static_cast<ScalarType>(y_max() - start_pos + 1) /
            static_cast<ScalarType>(y_max() - y_min() + 1) >
            kCloseToBboxPercentage) {
            start_pos = y_max();
        }
        if (static_cast<ScalarType>(end_pos - y_min() + 1) /
            static_cast<ScalarType>(y_max() - y_min() + 1) >
            kCloseToBboxPercentage) {
            end_pos = y_min();
        }
        splitContourVertical(split_len, true, start_pos, end_pos);

        // split anticlockwise contour
        start_pos = static_cast<int>(_vertices->at(_anticlockwise_edge->start_vertex_id).y());
        end_pos = static_cast<int>(_vertices->at(_anticlockwise_edge->end_vertex_id).y());
        if (static_cast<ScalarType>(y_max() - start_pos + 1) /
            static_cast<ScalarType>(y_max() - y_min() + 1) >
            kCloseToBboxPercentage) {
            start_pos = y_max();
        }
        if (static_cast<ScalarType>(end_pos - y_min() + 1) /
            static_cast<ScalarType>(y_max() - y_min() + 1) >
            kCloseToBboxPercentage) {
            end_pos = y_min();
        }
        splitContourVertical(split_len, false, start_pos, end_pos);

    } else if (_bbox.split == BoundingBoxSplitType::HORIZONTAL) {
        // compute split bin size in horizontal direction
        split_len = static_cast<int>(static_cast<ScalarType>(split_len) *
                                     static_cast<ScalarType>(_bbox.width()) / bbox_diag_len);
        split_len = std::max(split_len, 5);

        // split clockwise contour
        int start_pos = static_cast<int>(_vertices->at(_clockwise_edge->start_vertex_id).x());
        int end_pos = static_cast<int>(_vertices->at(_clockwise_edge->end_vertex_id).x());
        bool is_left_to_right = start_pos <= end_pos;
        if (static_cast<ScalarType>(is_left_to_right ?
                                    start_pos - x_min() + 1 : x_max() - start_pos + 1) /
            static_cast<ScalarType>(x_max() - x_min() + 1) >
            kCloseToBboxPercentage) {
            start_pos = is_left_to_right ? x_min() : x_max();
        }
        if (static_cast<ScalarType>(is_left_to_right ?
                                    x_max() - end_pos + 1 : end_pos - x_min() + 1) /
            static_cast<ScalarType>(x_max() - x_min() + 1) >
            kCloseToBboxPercentage) {
            end_pos = is_left_to_right ? x_max() : x_min();
        }
        splitContourHorizontal(split_len, true, start_pos, end_pos);

        // split anticlockwise contour
        start_pos = static_cast<int>(_vertices->at(_anticlockwise_edge->start_vertex_id).x());
        end_pos = static_cast<int>(_vertices->at(_anticlockwise_edge->end_vertex_id).x());
        is_left_to_right = start_pos <= end_pos;
        if (static_cast<ScalarType>(is_left_to_right ?
                                    start_pos - x_min() + 1 : x_max() - start_pos + 1) /
            static_cast<ScalarType>(x_max() - x_min() + 1) >
            kCloseToBboxPercentage) {
            start_pos = is_left_to_right ? x_min() : x_max();
        }
        if (static_cast<ScalarType>(is_left_to_right ?
                                    x_max() - end_pos + 1 : end_pos - x_min() + 1) /
            static_cast<ScalarType>(x_max() - x_min() + 1) >
            kCloseToBboxPercentage) {
            end_pos = is_left_to_right ? x_max() : x_min();
        }
        splitContourHorizontal(split_len, false, start_pos, end_pos);

    } else {
        LOG(FATAL) << "unknown bounding box split type: " << _bbox.split;
    }

    // check the first and last edges of clockwise and anticlockwise contours
    bool is_first_edge_close
        = (_vertices->at(_clockwise_edges->at(0).start_vertex_id) -
           _vertices->at(_anticlockwise_edges->at(0).start_vertex_id)).norm() <= kCloseEdgeLength;
    bool is_last_edge_close
        = (_vertices->at(_clockwise_edges->back().end_vertex_id) -
           _vertices->at(_anticlockwise_edges->back().end_vertex_id)).norm() <= kCloseEdgeLength;
    if (is_first_edge_close && is_last_edge_close) {
        if (_clockwise_edges->size() >= 3) {
            _clockwise_edges->erase(_clockwise_edges->begin());
            _clockwise_edges->pop_back();
        }
        if (_anticlockwise_edges->size() >= 3) {
            _anticlockwise_edges->erase(_anticlockwise_edges->begin());
            _anticlockwise_edges->pop_back();
        }
    } else if (is_first_edge_close) {
        if (_clockwise_edges->size() >= 2) {
            _clockwise_edges->erase(_clockwise_edges->begin());
        }
        if (_anticlockwise_edges->size() >= 2) {
            _anticlockwise_edges->erase(_anticlockwise_edges->begin());
        }
    } else if (is_last_edge_close) {
        if (_clockwise_edges->size() >= 2) {
            _clockwise_edges->pop_back();
        }
        if (_anticlockwise_edges->size() >= 2) {
            _anticlockwise_edges->pop_back();
        }
    }
}
#endif

void ConnectedComponent::process(ScalarType split_siz, int split_len) {
    findBboxPixels();
    findVertices();
    findEdges();
    if (determineSplit(split_siz) != ConnectedComponent::BoundingBoxSplitType::NONE) {
        findContourForSplit();
        splitContour(split_len);
    }
}

/** split a CC into several smaller ones **/
vector<int> ConnectedComponent::get_split_ranges(int siz, int len_split) {
    if (siz <= 0) {
        XLOG(ERROR) << "siz should be a positive number: " << siz;
    }
    if (len_split <= 0) {
        XLOG(ERROR) << "len_split should be a positive number: " << len_split;
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

/** find connected components **/
bool find_cc(const cv::Mat& src, vector<shared_ptr<ConnectedComponent> >& cc) {
    if (src.empty()) {
        XLOG(ERROR) << "input image is empty";
        return false;
    }

    if (src.type() != CV_8UC1) {
        XLOG(ERROR) << "input image type is not CV_8UC1";
        return false;
    }
    cc.clear();
    size_t total_pix = src.total();

    std::vector<int> frame_label(total_pix);
    DisjointSet labels(total_pix);
    std::vector<int> root_map;
    root_map.reserve(total_pix);

    int x = 0;
    int y = 0;
    const uchar *cur_p = NULL;
    const uchar *prev_p = src.ptr<uchar>(0);
    int left_val = 0;
    int up_val = 0;
    int cur_idx = 0;
    int left_idx = 0;
    int up_idx = 0;

    // first loop logic
    for (y = 0; y < src.rows; y++) {
        cur_p = src.ptr<uchar>(y);
        for (x = 0; x < src.cols; x++, cur_idx++) {
            left_idx = cur_idx - 1;
            up_idx = cur_idx - src.size().width;

            left_val = (x == 0) ? 0 : cur_p[x - 1];
            up_val = (y == 0) ? 0 : prev_p[x];

            if (cur_p[x] != 0) {
                if (left_val == 0 && up_val == 0) {
                    // current pixel is foreground and has no connected neighbors
                    frame_label[cur_idx] = labels.add();
                    //root_map[frame_label[cur_idx]] = -1;
                    root_map.push_back(-1);
                } else if (left_val != 0 && up_val == 0) {
                    // current pixel is foreground and has left neighbor connected
                    frame_label[cur_idx] = frame_label[left_idx];
                } else if (left_val == 0 && up_val != 0) {
                    // current pixel is foreground and has up neighbor connect
                    frame_label[cur_idx] = frame_label[up_idx];
                } else {
                    // current pixel is foreground and is connected to left and up neighbors
                    frame_label[cur_idx] = (frame_label[left_idx] > frame_label[up_idx])
                                           ? frame_label[up_idx] : frame_label[left_idx];
                    labels.unite(frame_label[left_idx], frame_label[up_idx]);
                }
            } else {
                frame_label[cur_idx] = -1;
            }
        }  // end for x
        prev_p = cur_p;
    }  // end for y
    if (root_map.size() != labels.num()) {
        XLOG(ERROR) << "the size of root map and labels are not equal.";
        return false;
    }
    XLOG(INFO) << "subset number = " << labels.size();

    // second loop logic
    cur_idx = 0;
    int curt_label = 0;
    int cc_count = 0;
    for (y = 0; y < src.size().height; y++) {
        for (x = 0; x < src.size().width; x++, cur_idx++) {
            curt_label = frame_label[cur_idx];
            if (curt_label < -1) {
                XLOG(ERROR) << "root_map[curt_label] should be no less than -1: "
                            << root_map.at(curt_label);
                return false;
            }
            if (curt_label >= 0) {
                if (curt_label >= static_cast<int>(labels.num())) {
                    XLOG(ERROR) << "curt_label should be smaller than labels.num(): "
                                << curt_label << " vs. " << labels.num();
                    return false;
                }
                curt_label = labels.find(curt_label);
                if (curt_label >= static_cast<int>(root_map.size())) {
                    XLOG(ERROR) << "curt_label should be smaller than root_map.size() "
                                << curt_label << " vs. " << root_map.size();
                    return false;
                }
                if (root_map.at(curt_label) != -1) {
                    cc[root_map.at(curt_label)]->addPixel(x, y);
                } else {
                    cc.push_back(std::make_shared<ConnectedComponent>(x, y));
                    root_map.at(curt_label) = cc_count++;
                }
            }
        }  // end for x
    }   // end for y
    XLOG(INFO) << "cc number = " << cc_count;

    return true;
}

bool find_cc(const cv::Mat &src,
             const cv::Rect &roi,
             vector<shared_ptr<ConnectedComponent> >& cc) {
    if (src.empty()) {
        XLOG(ERROR) << "input image is empty";
        return false;
    }

    if (src.type() != CV_8UC1) {
        XLOG(ERROR) << "input image type is not CV_8UC1";
        return false;
    }
    cc.clear();

    int x_min = roi.x;
    int y_min = roi.y;
    int x_max = x_min + roi.width - 1;
    int y_max = y_min + roi.height - 1;
    if (x_min < 0) {
        XLOG(ERROR) << "x_min is less than zero: " << x_min;
    }
    if (y_min < 0) {
        XLOG(ERROR) << "y_min is less than zero: " << y_min;
    }
    if (x_max >= src.size().width) {
        XLOG(ERROR) << "x_max is larger than image width: " << x_max << "|" << src.cols;
    }
    if (y_max >= src.size().height) {
        XLOG(ERROR) << "y_max is larger than image height: " << y_max << "|" << src.rows;
    }

    size_t total_pix = static_cast<size_t>(roi.width * roi.height);

    std::vector<int> frame_label(total_pix);
    DisjointSet labels(total_pix);
    std::vector<int> root_map;
    root_map.reserve(total_pix);

    int x = 0;
    int y = 0;
    const uchar *cur_p = NULL;
    const uchar *prev_p = src.ptr<uchar>(y_min);
    int left_val = 0;
    int up_val = 0;
    int cur_idx = 0;
    int left_idx = 0;
    int up_idx = 0;

    // first loop logic
    for (y = y_min; y <= y_max; y++) {
        cur_p = src.ptr<uchar>(y);
        for (x = x_min; x <= x_max; x++, cur_idx++) {
            left_idx = cur_idx - 1;
            up_idx = cur_idx - src.size().width;

            if (x == x_min) {
                left_val = 0;
            } else {
                left_val = cur_p[x - 1];
            }

            if (y == y_min) {
                up_val = 0;
            } else {
                up_val = prev_p[x];
            }

            if (cur_p[x] > 0) {
                if (left_val == 0 && up_val == 0) {
                    // current pixel is foreground and has no connected neighbors
                    frame_label[cur_idx] = labels.add();
                    //root_map[frame_label[cur_idx]] = -1;
                    root_map.push_back(-1);
                } else if (left_val != 0 && up_val == 0) {
                    // current pixel is foreground and has left neighbor connected
                    frame_label[cur_idx] = frame_label[left_idx];
                } else if (left_val == 0 && up_val != 0) {
                    // current pixel is foreground and has up neighbor connect
                    frame_label[cur_idx] = frame_label[up_idx];
                } else {
                    // current pixel is foreground and is connected to left and up neighbors
                    frame_label[cur_idx] = (frame_label[left_idx] > frame_label[up_idx])
                                           ? frame_label[up_idx]
                                           : frame_label[left_idx];
                    labels.unite(frame_label[left_idx], frame_label[up_idx]);
                }
            } else {
                frame_label[cur_idx] = -1;
            }
        }  // end for x
        prev_p = cur_p;
    }  // end for y
    if (root_map.size() != labels.num()) {
        XLOG(ERROR) << "the size of root map and labels are not equal.";
        return false;
    }
    XLOG(INFO) << "subset number = " << labels.size();

    // second loop logic
    cur_idx = 0;
    int curt_label = 0;
    int cc_count = 0;
    for (y = y_min; y <= y_max; y++) {
        for (x = x_min; x <= x_max; x++, cur_idx++) {
            curt_label = frame_label[cur_idx];
            if (curt_label < -1) {
                XLOG(ERROR) << "curt_label should be no less than -1: " << curt_label;
                return false;
            }
            if (curt_label >= 0) {
                if (curt_label >= static_cast<int>(labels.num())) {
                    XLOG(ERROR) << "curt_label should be smaller than labels.num(): "
                                << curt_label << " vs. " << labels.num();
                    return false;
                }
                curt_label = labels.find(curt_label);
                if (curt_label >= static_cast<int>(root_map.size())) {
                    XLOG(ERROR) << "curt_label should be smaller than root_map.size() "
                                << curt_label << " vs. " << root_map.size();
                    return false;
                }
                if (root_map.at(curt_label) != -1) {
                    cc[root_map.at(curt_label)]->addPixel(x, y);
                } else {
                    cc.push_back(std::make_shared<ConnectedComponent>(x, y));
                    root_map.at(curt_label) = cc_count++;
                }
            }
        }  // end for x
    }   // end for y
    XLOG(INFO) << "cc number = " << cc_count;

    return true;
}

bool find_cc_block(const cv::Mat &src, const cv::Rect &roi,
                   vector<shared_ptr<ConnectedComponent> >& cc) {
    XLOG(INFO) << "using CUDA to find CC ...";
    if (src.empty()) {
        XLOG(ERROR) << "input image is empty";
        return false;
    }

    if (src.type() != CV_8UC1) {
        XLOG(ERROR) << "input image type is not CV_8UC1";
        return false;
    }
    cc.clear();

    int x_min = roi.x;
    int y_min = roi.y;
    int x_max = x_min + roi.width - 1;
    int y_max = y_min + roi.height - 1;
    if (x_min < 0) {
        XLOG(ERROR) << "x_min is less than zero: " << x_min;
    }
    if (y_min < 0) {
        XLOG(ERROR) << "y_min is less than zero: " << y_min;
    }
    if (x_max >= src.size().width) {
        XLOG(ERROR) << "x_max is larger than image width: " << x_max << "|" << src.cols;
    }
    if (y_max >= src.size().height) {
        XLOG(ERROR) << "y_max is larger than image height: " << y_max << "|" << src.rows;
    }

    size_t total_pix = static_cast<size_t>(roi.width * roi.height);

    const uchar* img = src.data + y_min * src.cols + x_min;

    int* labels = static_cast<int*>(malloc(total_pix * sizeof(int)));
    block_union_find_cuda(img, roi.width, roi.height, src.cols, labels);

    int cur_idx = 0;
    int curt_label = 0;
    int cc_count = 0;
    vector<int> root_map(total_pix, -1);
    for (int y = y_min; y <= y_max; y++) {
        for (int x = x_min; x <= x_max; x++, cur_idx++) {
            curt_label = labels[cur_idx];

            if (curt_label >= 0) {
                if (curt_label >= static_cast<int>(total_pix)) {
                    XLOG(ERROR) << "curt_label should be smaller than root_map.size() "
                                << curt_label << " vs. " << total_pix;
                    return false;
                }
                if (root_map[curt_label] != -1) {
                    cc[root_map[curt_label]]->addPixel(x, y);
                } else {
                    cc.push_back(std::make_shared<ConnectedComponent>(x, y));
                    root_map[curt_label] = cc_count++;
                }
            }
        }  // end for x
    }   // end for y

    free(labels);

    XLOG(INFO) << "succ. to find CC: #cc number = " << cc_count;

    return true;
}

ConnectedComponentGenerator::ConnectedComponentGenerator(int image_width, int image_height)
        : _image_width(image_width),
          _image_height(image_height),
          _width(image_width),
          _height(image_height),
          _roi_x_min(0),
          _roi_y_min(0),
          _roi_x_max(image_width - 1),
          _roi_y_max(image_height - 1) {
        _total_pix = static_cast<size_t>(_image_width) * static_cast<size_t>(_image_height);
#if _CUDA_CC
        cudaChannelFormatDesc uchar_desc = cudaCreateChannelDesc<unsigned char>();
        cudaMallocArray(&_img_array, &uchar_desc,
                        static_cast<size_t>(_width),
                        static_cast<size_t>(_height));
        cudaBindTextureToArray(img_tex, _img_array, uchar_desc);
        cudaMalloc((void**) &_label_array,
                   static_cast<size_t>(_width) * static_cast<size_t>(_height) * sizeof(int));
        _labels = static_cast<int*>(malloc(_total_pix * sizeof(int)));
#else
        _labels.init(_total_pix);
        _frame_label.resize(_total_pix, -1);
#endif
        _root_map.reserve(_total_pix);
}

ConnectedComponentGenerator::ConnectedComponentGenerator(int image_width, int image_height, cv::Rect roi)
        : _image_width(image_width),
          _image_height(image_height),
          _width(roi.width),
          _height(roi.height),
          _roi_x_min(roi.x),
          _roi_y_min(roi.y),
          _roi_x_max(roi.x + roi.width - 1),
          _roi_y_max(roi.y + roi.height - 1) {
        if (_roi_x_min < 0) {
            XLOG(ERROR) << "x_min is less than zero: " << _roi_x_min;
        }
        if (_roi_y_min < 0) {
            XLOG(ERROR) << "y_min is less than zero: " << _roi_y_min;
        }
        if (_roi_x_max >= _image_width) {
            XLOG(ERROR) << "x_max is larger than image width: "
                        << _roi_x_max << "|" << _image_width;
        }
        if (_roi_y_max >= _image_height) {
            XLOG(ERROR) << "y_max is larger than image height: "
                        << _roi_y_max << "|" << _image_height;
        }
        _total_pix = static_cast<size_t>(_width) * static_cast<size_t>(_height);
#if _CUDA_CC
        cudaChannelFormatDesc uchar_desc = cudaCreateChannelDesc<unsigned char>();
        _img_array = NULL;
        cudaMallocArray(&_img_array, &uchar_desc,
                        static_cast<size_t>(_width),
                        static_cast<size_t>(_height));
        cudaBindTextureToArray(img_tex, _img_array, uchar_desc);

        cudaMalloc((void**) &_label_array,
                   static_cast<size_t>(_width) * static_cast<size_t>(_height) * sizeof(int));

        cudaError_t cuda_err = cudaGetLastError();
        if (cuda_err != cudaSuccess) {
            XLOG(ERROR) << "failed to initialize 'img_array' and 'label_array' with CUDA: "
                        << cudaGetErrorString(cuda_err);
        }

        _labels = static_cast<int*>(malloc(_total_pix * sizeof(int)));
#else
        _labels.init(_total_pix);
        _frame_label.resize(_total_pix, -1);
#endif
        _root_map.reserve(_total_pix);
}

#if _CUDA_CC
bool ConnectedComponentGenerator::block_union_find(const unsigned char* img) {
    cudaError_t cuda_err;

    if (_width == _image_width) {
        size_t siz =
            static_cast<size_t>(_width) * static_cast<size_t>(_height) * sizeof(unsigned char);
        cudaMemcpyToArray(_img_array, 0, 0, img, siz, cudaMemcpyHostToDevice);
    } else {
        size_t siz = static_cast<size_t>(_width) * sizeof(unsigned char);
        for (size_t i = 0; i < static_cast<size_t>(_height); ++i) {
            cudaMemcpyToArray(_img_array, 0, i, img, siz, cudaMemcpyHostToDevice);
            img += _image_width;
        }
    }

    dim3 block(UF_BLOCK_WIDTH, UF_BLOCK_HEIGHT);
    dim3 grid(static_cast<unsigned int>((_width + UF_BLOCK_WIDTH - 1) / UF_BLOCK_WIDTH),
              static_cast<unsigned int>((_height + UF_BLOCK_HEIGHT - 1) / UF_BLOCK_HEIGHT));

    cuda_err = cudaGetLastError();
    if (cuda_err != cudaSuccess) {
        XLOG(ERROR) << "failed to start block union find with CUDA: "
                    << cudaGetErrorString(cuda_err);
        return false;
    }

    cudaThreadSetCacheConfig(cudaFuncCachePreferShared);

    UF_block_internal<<<grid, block>>>(_label_array, _width, _height);

    cudaThreadSetCacheConfig(cudaFuncCachePreferL1);

    UF_block_boundary<<<grid, block>>>(_label_array, _width, _height);

    UF_find_root<<<grid, block>>>(_label_array, _width, _height);

    cudaMemcpy(_labels, _label_array,
               static_cast<size_t>(_width) * static_cast<size_t>(_height) * sizeof(int),
               cudaMemcpyDeviceToHost);

    cuda_err = cudaGetLastError();
    if (cuda_err != cudaSuccess) {
        XLOG(ERROR) << "failed to finish block union find with CUDA: "
                    << cudaGetErrorString(cuda_err);
        return false;
    }

    return true;
}

bool ConnectedComponentGenerator::find_cc(const cv::Mat& lane_map,
                                          vector<shared_ptr<ConnectedComponent> >& cc) {
    XLOG(INFO) << "using CUDA to find CC ...";
    if (lane_map.empty()) {
        XLOG(ERROR) << "input lane map is empty";
        return false;
    }
    if (lane_map.type() != CV_8UC1) {
        XLOG(ERROR) << "input lane map type is not CV_8UC1";
        return false;
    }

    if (lane_map.cols != _image_width) {
        XLOG(ERROR) << "The width of input lane map does not match";
    }
    if (lane_map.rows != _image_height) {
        XLOG(ERROR) << "The height of input lane map does not match";
    }

    cc.clear();

    const unsigned char* img = lane_map.data + _roi_y_min * _image_width + _roi_x_min;

    //int* labels = static_cast<int*>(malloc(_total_pix * sizeof(int)));
    block_union_find(img);

    int cur_idx = 0;
    int curt_label = 0;
    int cc_count = 0;
    _root_map.assign(_total_pix, -1);
    for (int y = _roi_y_min; y <= _roi_y_max; ++y) {
        for (int x = _roi_x_min; x <= _roi_x_max; ++x) {
            curt_label = _labels[cur_idx];

            if (curt_label >= 0) {
                if (curt_label >= static_cast<int>(_total_pix)) {
                    XLOG(ERROR) << "curt_label should be smaller than root_map.size() "
                                << curt_label << " vs. " << _total_pix;
                    return false;
                }
                if (_root_map[curt_label] != -1) {
                    cc[_root_map[curt_label]]->addPixel(x, y);
                } else {
                    cc.push_back(std::make_shared<ConnectedComponent>(x, y));
                    _root_map[curt_label] = cc_count++;
                }
            }

            ++cur_idx;
        }  // end for x
    }   // end for y

    XLOG(INFO) << "succ. to find CC: #cc number = " << cc_count;

    return true;
}

#else

bool ConnectedComponentGenerator::find_cc(const cv::Mat& lane_map,
                                          vector<shared_ptr<ConnectedComponent> >& cc) {
    if (lane_map.empty()) {
        XLOG(ERROR) << "input lane map is empty";
        return false;
    }
    if (lane_map.type() != CV_8UC1) {
        XLOG(ERROR) << "input lane map type is not CV_8UC1";
        return false;
    }

    if (lane_map.cols != _image_width) {
        XLOG(ERROR) << "The width of input lane map does not match";
    }
    if (lane_map.rows != _image_height) {
        XLOG(ERROR) << "The height of input lane map does not match";
    }

    cc.clear();

    _labels.reset();
    _root_map.clear();

    int x = 0;
    int y = 0;
    const uchar *cur_p = NULL;
    const uchar *prev_p = lane_map.ptr<uchar>(_roi_y_min);
    int left_val = 0;
    int up_val = 0;
    int cur_idx = 0;
    int left_idx = 0;
    int up_idx = 0;

    // first loop logic
    for (y = _roi_y_min; y <= _roi_y_max; y++) {
        cur_p = lane_map.ptr<uchar>(y);
        for (x = _roi_x_min; x <= _roi_x_max; x++, cur_idx++) {
            left_idx = cur_idx - 1;
            up_idx = cur_idx - _image_width;

            if (x == _roi_x_min) {
                left_val = 0;
            } else {
                left_val = cur_p[x - 1];
            }

            if (y == _roi_y_min) {
                up_val = 0;
            } else {
                up_val = prev_p[x];
            }

            if (cur_p[x] > 0) {
                if (left_val == 0 && up_val == 0) {
                    // current pixel is foreground and has no connected neighbors
                    _frame_label[cur_idx] = _labels.add();
                    _root_map.push_back(-1);
                } else if (left_val != 0 && up_val == 0) {
                    // current pixel is foreground and has left neighbor connected
                    _frame_label[cur_idx] = _frame_label[left_idx];
                } else if (left_val == 0 && up_val != 0) {
                    // current pixel is foreground and has up neighbor connect
                    _frame_label[cur_idx] = _frame_label[up_idx];
                } else {
                    // current pixel is foreground and is connected to left and up neighbors
                    _frame_label[cur_idx] = (_frame_label[left_idx] > _frame_label[up_idx])
                                           ? _frame_label[up_idx]
                                           : _frame_label[left_idx];
                    _labels.unite(_frame_label[left_idx], _frame_label[up_idx]);
                }
            } else {
                _frame_label[cur_idx] = -1;
            }
        }  // end for x
        prev_p = cur_p;
    }  // end for y
    if (_root_map.size() != _labels.num()) {
        XLOG(ERROR) << "the size of root map and labels are not equal.";
        return false;
    }
    XLOG(INFO) << "subset number = " << _labels.size();

    // second loop logic
    cur_idx = 0;
    int curt_label = 0;
    int cc_count = 0;
    for (y = _roi_y_min; y <= _roi_y_max; y++) {
        for (x = _roi_x_min; x <= _roi_x_max; x++, cur_idx++) {
            curt_label = _frame_label[cur_idx];
            //if (curt_label < -1) {
            //    XLOG(ERROR) << "curt_label should be no less than -1: " << curt_label;
            //    return false;
            //}
            if (curt_label >= 0) {
                if (curt_label >= static_cast<int>(_labels.num())) {
                    XLOG(ERROR) << "curt_label should be smaller than labels.num(): "
                                << curt_label << " vs. " << _labels.num();
                    return false;
                }
                curt_label = _labels.find(curt_label);
                if (curt_label >= static_cast<int>(_root_map.size())) {
                    XLOG(ERROR) << "curt_label should be smaller than root_map.size() "
                                << curt_label << " vs. " << _root_map.size();
                    return false;
                }
                if (_root_map[curt_label] != -1) {
                    cc[_root_map[curt_label]]->addPixel(x, y);
                } else {
                    cc.push_back(std::make_shared<ConnectedComponent>(x, y));
                    _root_map[curt_label] = cc_count++;
                }
            }
        }  // end for x
    }   // end for y
    XLOG(INFO) << "cc number = " << cc_count;

    return true;
}
#endif

}  // namespace lane_post_process
}  // namespace obstacle
}  // namespace perception
}  // namespace adu
