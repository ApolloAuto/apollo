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

// @brief: connected component analysis for lane detection

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_CONNECTED_COMPONENTS_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_CONNECTED_COMPONENTS_H_

#include <memory>
#include <vector>
#include <string>
#include <unordered_set>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include <cuda.h>
#include <cuda_runtime.h>

#include <xlog.h>

#include "type.hpp"

namespace adu {
namespace perception {
namespace obstacle {
namespace lane_post_process {

#ifndef NUM_RESERVE_VERTICES
#define NUM_RESERVE_VERTICES 4
#endif

#ifndef NUM_RESERVE_EDGES
#define NUM_RESERVE_EDGES 6
#endif

#define _CUDA_CC true

class DisjointSet {
public:
    DisjointSet() : m_disjoint_array(), m_subset_num(0) {}
    DisjointSet(size_t size) : m_disjoint_array(), m_subset_num(0) {
        m_disjoint_array.reserve(size);
    }
    ~DisjointSet() {}

    void init(size_t size) {
        m_disjoint_array.clear();
        m_disjoint_array.reserve(size);
        m_subset_num = 0;
    }

    void reset() {
        m_disjoint_array.clear();
        m_subset_num = 0;
    }

    int add();        // add a new element, which is a subset by itself;
    int find(int x);  // return the root of x
    void unite(int x, int y);
    int size() const { return m_subset_num; }
    size_t num() const { return m_disjoint_array.size(); }

private:
    std::vector<int> m_disjoint_array;
    int m_subset_num;
};

class ConnectedComponent {
public:
    typedef Eigen::Matrix<ScalarType, 2, 1> Vertex;
    typedef Eigen::Matrix<ScalarType, 2, 1> Displacement;
    enum BoundingBoxSplitType {
        NONE = -1,       // without split
        VERTICAL,        // split in vertical direction (y)
        HORIZONTAL,      // split in horizontal direction (x)
    };

    struct Edge {
        int start_vertex_id;
        int end_vertex_id;
        Displacement vec;
        ScalarType len;
        ScalarType orie;
        Edge ()
            : start_vertex_id(-1),
              end_vertex_id(-1),
              vec(0.0, 0.0),
              len(0.0),
              orie(0.0) {}

        int get_start_vertex_id() const {
            return start_vertex_id;
        }
        int get_end_vertex_id() const {
            return end_vertex_id;
        }
    };

    struct BoundingBox {
        int x_min;  // left
        int y_min;  // up
        int x_max;  // right
        int y_max;  // down
        std::shared_ptr<std::vector<int> > bbox_pixel_idx;
        BoundingBoxSplitType split;
        std::shared_ptr<std::vector<int> > left_contour;
        std::shared_ptr<std::vector<int> > up_contour;
        std::shared_ptr<std::vector<int> > right_contour;
        std::shared_ptr<std::vector<int> > down_contour;

        BoundingBox ()
            : x_min(-1), y_min(-1), x_max(-1), y_max(-1),
              split(BoundingBoxSplitType::NONE) {
            bbox_pixel_idx = std::make_shared<std::vector<int> >();
            left_contour = std::make_shared<std::vector<int> >();
            up_contour = std::make_shared<std::vector<int> >();
            right_contour = std::make_shared<std::vector<int> >();
            down_contour = std::make_shared<std::vector<int> >();
        }

        BoundingBox (int x, int y)
            : x_min(x), y_min(y), x_max(x), y_max(y),
              split(BoundingBoxSplitType::NONE) {
            bbox_pixel_idx = std::make_shared<std::vector<int> >();
            left_contour = std::make_shared<std::vector<int> >();
            up_contour = std::make_shared<std::vector<int> >();
            right_contour = std::make_shared<std::vector<int> >();
            down_contour = std::make_shared<std::vector<int> >();
        }

        int width() const {
            return x_max - x_min + 1;
        }

        int height() const {
            return y_max - y_min + 1;
        }
    };

    ConnectedComponent()
        : _pixel_count(0), _bbox() {
        _pixels = std::make_shared<std::vector<cv::Point2i> >();
        _vertices = std::make_shared<std::vector<Vertex> >();
        _vertices->reserve(NUM_RESERVE_VERTICES);
        _edges = std::make_shared<std::vector<Edge> >();
        _edges->reserve(NUM_RESERVE_EDGES);
        _max_len_edge_id = -1;
        _clockwise_edge = std::make_shared<Edge>();
        _anticlockwise_edge = std::make_shared<Edge>();
        _inner_edge = std::make_shared<Edge>();
        _clockwise_edges = std::make_shared<std::vector<Edge> >();
        _anticlockwise_edges = std::make_shared<std::vector<Edge> >();
        _inner_edges = std::make_shared<std::vector<Edge> >();
    }

    ConnectedComponent(int x, int y)
        : _pixel_count(1), _bbox(x, y) {
        _pixels = std::make_shared<std::vector<cv::Point2i> >();
        _pixels->push_back(cv::Point(x, y));
        _vertices = std::make_shared<std::vector<Vertex> >();
        _vertices->reserve(NUM_RESERVE_VERTICES);
        _edges = std::make_shared<std::vector<Edge> >();
        _edges->reserve(NUM_RESERVE_EDGES);
        _max_len_edge_id = -1;
        _clockwise_edge = std::make_shared<Edge>();
        _anticlockwise_edge = std::make_shared<Edge>();
        _inner_edge = std::make_shared<Edge>();
        _clockwise_edges = std::make_shared<std::vector<Edge> >();
        _anticlockwise_edges = std::make_shared<std::vector<Edge> >();
        _inner_edges = std::make_shared<std::vector<Edge> >();
    }

    ~ConnectedComponent() {}

    // CC pixels
    void addPixel(int x, int y);
    int getPixelCount() const { return _pixel_count; }
    std::shared_ptr<const std::vector<cv::Point2i> > getPixels() const {
        return _pixels;
    }

    // bounding box
    const BoundingBox* bbox() const { return &_bbox; }
    int x_min() const { return _bbox.x_min; }
    int y_min() const { return _bbox.y_min; }
    int x_max() const { return _bbox.x_max; }
    int y_max() const { return _bbox.y_max; }

    cv::Rect getBoundingBox() const {
        return cv::Rect(_bbox.x_min, _bbox.y_min,
                        _bbox.x_max - _bbox.x_min + 1,
                        _bbox.y_max - _bbox.y_min + 1);
    }

    int getBoundingBoxArea() const {
        int area = (_bbox.x_max - _bbox.x_min + 1) * (_bbox.y_max - _bbox.y_min + 1);
#if _DEBUG
        CHECK_GE(area, 0);
#endif
        return area;
    }

    // split bounding box
    BoundingBoxSplitType determineSplit(ScalarType split_siz);

    void findContourForSplit();

    // bounding box pixels
    void findBboxPixels();

    std::shared_ptr<const std::vector<int> > bbox_pixel_idx() const {
        return _bbox.bbox_pixel_idx;
    }

    int getBboxPixelCount() const {
        return static_cast<int>(_bbox.bbox_pixel_idx->size());
    }

    // vertices
    void findVertices();
    std::shared_ptr<const std::vector<Vertex> > getVertices() const {
        return _vertices;
    }

    Vertex getVertex(int vertex_id) const {
        assert(vertex_id >= 0 && vertex_id < this->getVertexCount());
        return _vertices->at(vertex_id);
    }

    int getVertexCount() const { return static_cast<int>(_vertices->size()); }

    // edges
    bool isValidEdgeVertices(int i, int j) {
        return i >= 0 &&
               i < this->getVertexCount() &&
               j >= 0 &&
               j < this->getVertexCount() &&
               i != j;
    }

    void findEdges();
    int getEdgeCount() const { return static_cast<int>(_edges->size()); }
    const Edge* getMaxLenthEdge() const { return &_edges->at(_max_len_edge_id); }
    std::shared_ptr<const Edge> getClockWiseEdge() const { return _clockwise_edge; }
    std::shared_ptr<const Edge> getAntiClockWiseEdge() const { return _anticlockwise_edge; }
    std::shared_ptr<const Edge> getInnerEdge() const { return _inner_edge; }

    void splitContourVertical(int start_vertex_id, int end_vertex_id,
                              int len_split, bool is_clockwise);
    void splitContourVertical(int len_split, bool is_clockwise, int start_pos, int end_pos);
    void splitContourHorizontal(int start_vertex_id, int end_vertex_id,
                                int len_split, bool is_clockwise);
    void splitContourHorizontal(int len_split, bool is_clockwise, int start_pos, int end_pos);
    void splitContour(int split_len);
    std::shared_ptr<std::vector<Edge> > getClockWiseEdges() const { return _clockwise_edges; }
    std::shared_ptr<std::vector<Edge> > getAntiClockWiseEdges() const {
        return _anticlockwise_edges;
    }
    std::shared_ptr<std::vector<Edge> > getInnerEdges() const { return _inner_edges; }

    void process(ScalarType split_siz, int split_len);

private:
    int sub2ind(int row, int col, int width) {
        return row * width + col;
    }

    std::vector<int> get_split_ranges(int siz, int len_split);

    Edge makeEdge(int i, int j);

    int _pixel_count;
    std::shared_ptr<std::vector<cv::Point2i> > _pixels;
    BoundingBox _bbox;
    std::shared_ptr<std::vector<Vertex> > _vertices;
    std::shared_ptr<std::vector<Edge> > _edges;
    int _max_len_edge_id;
    std::shared_ptr<Edge> _clockwise_edge, _anticlockwise_edge;
    std::shared_ptr<Edge> _inner_edge;
    std::shared_ptr<std::vector<Edge> > _clockwise_edges, _anticlockwise_edges;
    std::shared_ptr<std::vector<Edge> > _inner_edges;
};

typedef std::shared_ptr<ConnectedComponent> ConnectedComponentPtr;
typedef const std::shared_ptr<ConnectedComponent> ConnectedComponentConstPtr;

bool find_cc(const cv::Mat &src, std::vector<std::shared_ptr<ConnectedComponent> > &cc);
bool find_cc(const cv::Mat &src, const cv::Rect &roi,
             std::vector<std::shared_ptr<ConnectedComponent> > &cc);
bool find_cc_block(const cv::Mat &src, const cv::Rect &roi,
                   std::vector<std::shared_ptr<ConnectedComponent> >& cc);

class ConnectedComponentGenerator {
public:
    ConnectedComponentGenerator(int image_width, int image_height);
    ConnectedComponentGenerator(int image_width, int image_height, cv::Rect roi);

    ~ConnectedComponentGenerator() {
#if _CUDA_CC
        cudaFree(_label_array);
        cudaFreeArray(_img_array);

        cudaError_t cuda_err = cudaGetLastError();
        if (cuda_err != cudaSuccess) {
            XLOG(ERROR) << "failed to release arrays '_label_array' and '_img_array' with CUDA: "
                        << cudaGetErrorString(cuda_err);
        }

        free(_labels);
#endif
    }

    bool find_cc(const cv::Mat& lane_map, std::vector<std::shared_ptr<ConnectedComponent> >& cc);

private:
#if _CUDA_CC
    bool block_union_find(const unsigned char* img);
#endif

private:
    size_t _total_pix;
    int _image_width;
    int _image_height;

    int _width;
    int _height;
    int _roi_x_min;
    int _roi_y_min;
    int _roi_x_max;
    int _roi_y_max;

#if _CUDA_CC
    int* _labels;
#else
    DisjointSet _labels;
    std::vector<int> _frame_label;
#endif
    std::vector<int> _root_map;
    cudaArray* _img_array;
    int* _label_array;
};

}  // namespace lane_post_process
}  // namespace obstacle
}  // namespace perception
}  // namespace adu

#endif  // ADU_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_CONNECTED_COMPONENTS_HPP
