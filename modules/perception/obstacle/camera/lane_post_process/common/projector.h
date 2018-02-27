// Copyright 2017 Baidu Inc. All Rights Reserved.
// Author: Zhu, Jun (zhujun08@baidu.com)
// brief: image space to ego-car ground space projection

#ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_PARSER_TRANSFORMER_TOOL_PROJECTOR_HPP
#define ADU_PERCEPTION_OBSTACLE_CAMERA_PARSER_TRANSFORMER_TOOL_PROJECTOR_HPP

//#include <yaml-cpp/yaml.h>
#include <cmath>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
//#include <boost/filesystem.hpp>

#include "lib/config_manager/calibration_config_manager.h"
#include "obstacle/camera/common/util.h"
#include "xlog.h"

namespace adu {
namespace perception {
namespace obstacle {
namespace transformer_tool {

#define _DEBUG false

template<typename T>
class Projector {
public:
    Projector();

    bool init(const cv::Rect &roi, const T &max_distance = 200.0, bool visualize = false);

    bool uv_2_xy(const T &u, const T &v, Eigen::Matrix<T, 2, 1> *p) const;

    bool uv_2_xy(const int &u, const int &v, Eigen::Matrix<T, 2, 1> *p) const{
        return uv_2_xy(static_cast<T>(u), static_cast<T>(v), p);
    }

    Eigen::Affine3d _camera_ex;
    Eigen::Affine3d _camera_to_ego_car;

    bool is_init() const { return _is_init; }

    bool is_valid_uv(const int &x, const int &y) const {
        return is_valid_uv(static_cast<T>(x), static_cast<T>(y));
    }

    bool is_valid_uv(const T &x, const T &y) const;

    bool is_valid_xy(const T &x, const T &y) const {
        return x >= _xy_xmin && x <= _xy_xmax && y >= _xy_ymin && y <= _xy_ymax;
    }

    bool is_vis() const { return _is_vis; }
    T x_step() const { return _x_step; }
    T y_step() const { return _y_step; }
    T xy_image_xmin() const { return _xy_image_xmin; }
    T xy_image_xmax() const { return _xy_image_xmax; }
    T xy_image_ymin() const { return _xy_image_ymin; }
    T xy_image_ymax() const { return _xy_image_ymax; }

    bool is_valid_xy_in_xy_image(const T &x, const T &y) const {
        return x >= _xy_image_xmin && x <= _xy_image_xmax &&
            y >= _xy_image_ymin && y <= _xy_image_ymax;
    }

    int xy_image_cols() const {
        CHECK(_is_init) << "Error: projector is not initialized.";
        return _xy_image_cols;
    }

    int xy_image_rows() const {
        CHECK(_is_init) << "Error: projector is not initialized.";
        return _xy_image_rows;
    }

    bool xy_2_xy_image_point(const T &x, const T &y, cv::Point *p) const;
    bool xy_2_xy_image_point(const Eigen::Matrix<T, 2, 1> &pos, cv::Point *p) const {
        return xy_2_xy_image_point(pos.x(), pos.y(), p);
    }

    bool uv_2_xy_image_point(const T &u, const T &v, cv::Point *p) const;

    bool uv_2_xy_image_point(const int &u, const int &v, cv::Point *p) const {
        return uv_2_xy_image_point(static_cast<T>(u), static_cast<T>(v), p);
    }

protected:
    bool project(T u, T v, Eigen::Matrix<T, 2, 1>* xy_point);

private:
    Eigen::Matrix<T, 3, 3> _trans_mat;

    // for visualizing IPM image
    std::vector<Eigen::Matrix<T, 2, 1> > _xy_grid;
    std::vector<cv::Point> _uv_2_xy_image;
    T _x_step;
    T _y_step;
    T _xy_image_xmin;
    T _xy_image_xmax;
    int _xy_image_cols;
    T _xy_image_ymin;
    T _xy_image_ymax;
    int _xy_image_rows;

    // ROI limits in IPM space
    T _xy_xmin;
    T _xy_ymin;
    T _xy_xmax;
    T _xy_ymax;

    // ROI bounds in image space
    T _uv_xmin;
    T _uv_ymin;
    T _uv_xmax;
    T _uv_ymax;

    T _uv_roi_x_step;   // the horizontal direction sampling step of ROI in image space
    T _uv_roi_y_step;   // the vertical direction sampling step of ROI in image space
    int _uv_roi_cols;   // the column number of ROI in image space
    int _uv_roi_rows;   // the row number of ROI in image space
    int _uv_roi_count;  // the count of ROI sample points

    std::vector<bool> _valid_xy;
    std::vector<Eigen::Matrix<T, 2, 1> > _uv_2_xy;

    bool _is_init;
    bool _is_vis;
};

template<typename T>
Projector<T>::Projector() {
    _xy_xmin = std::numeric_limits<T>::max();
    _xy_ymin = std::numeric_limits<T>::max();
    _xy_xmax = -std::numeric_limits<T>::max();
    _xy_ymax = -std::numeric_limits<T>::max();

    _uv_xmin = static_cast<T>(0);
    _uv_ymin = static_cast<T>(0);
    _uv_xmax = static_cast<T>(0);
    _uv_ymax = static_cast<T>(0);

    _uv_roi_x_step = static_cast<T>(1);
    _uv_roi_y_step = static_cast<T>(1);

    _uv_roi_cols = static_cast<int>((_uv_xmax - _uv_xmin) / _uv_roi_x_step) + 1;
    _uv_roi_rows = static_cast<int>((_uv_ymax - _uv_ymin) / _uv_roi_y_step) + 1;
    _uv_roi_count = _uv_roi_rows * _uv_roi_cols;

    _is_vis = false;
    _x_step = static_cast<T>(0.1);
    _y_step = static_cast<T>(0.1);

    _is_init = false;
}

template<typename T>
bool Projector<T>::init(const cv::Rect &roi, const T &max_distance, bool visualize) {
    XLOG(INFO) << "Initialize projector ...";

    // read transformation matrix from calibration config manager
    config_manager::CalibrationConfigManager* calibration_config_manager
        = base::Singleton<config_manager::CalibrationConfigManager>::get();

    const config_manager::CameraCalibrationPtr camera_calibration
        = calibration_config_manager->get_camera_calibration();

    _trans_mat = camera_calibration->get_camera2car_homography_mat().cast<T>();
#if _DEBUG
    XLOG(INFO) << "homography matirx: ";
    for (int i = 0; i < 3; ++i) {
        XLOG(INFO) << "              " << _trans_mat(i, 0) << ", "
                                       << _trans_mat(i, 1) << ", "
                                       << _trans_mat(i, 2) << "\n";
    }
#endif

    // set ROI
    _uv_xmin = static_cast<T>(roi.x);
    if (_uv_xmin < 0) {
        XLOG(ERROR) << "invalid ROI x min: " << roi.x;
        return false;
    }

    _uv_ymin = static_cast<T>(roi.y);
    if (_uv_ymin < 0) {
        XLOG(ERROR) << "invalid ROI y min: " << roi.y;
        return false;
    }

    _uv_xmax = static_cast<T>(roi.x + roi.width - 1);
    //if (_uv_xmax >= _image_width) {
    //    XLOG(ERROR) << "invalid ROI x max: " << roi.x + roi.width - 1;
    //    return false;
    //}
    if (_uv_xmax < 0) {
        XLOG(ERROR) << "invalid ROI x max: " << roi.x + roi.width - 1;
        return false;
    }

    _uv_ymax = static_cast<T>(roi.y + roi.height - 1);
    //if (_uv_ymax >= _image_height) {
    //    XLOG(ERROR) << "invalid ROI x max: " << roi.y + roi.height - 1;
    //    return false;
    //}
    if (_uv_ymax < 0) {
        XLOG(ERROR) << "invalid ROI y max: " << roi.y + roi.height - 1;
        return false;
    }

    XLOG(INFO) << "ROI in image space: "
               << "x_min=" << _uv_xmin << ", "
               << "x_max=" << _uv_xmax << ", "
               << "y_min=" << _uv_ymin << ", "
               << "y_max=" << _uv_ymax << ".";
    XLOG(INFO) << "ROI width = " << roi.width << ", height = " << roi.height;

    _uv_roi_cols = static_cast<int>((_uv_xmax - _uv_xmin) / _uv_roi_x_step) + 1;
    _uv_roi_rows = static_cast<int>((_uv_ymax - _uv_ymin) / _uv_roi_y_step) + 1;
    _uv_roi_count = _uv_roi_rows * _uv_roi_cols;
    XLOG(INFO) << "sampling step_x (u) = " << _uv_roi_x_step << ", "
               << "sampling step_y (v) = " << _uv_roi_y_step;
    XLOG(INFO) << "ROI #columns = " << _uv_roi_cols << ", " << "#rows = " << _uv_roi_rows;

    // do point-wise transform of ROI from image space to ground space
    _valid_xy.assign(_uv_roi_count, true);
    _uv_2_xy.resize(_uv_roi_count);
    int count_fail_points = 0;
    int count_too_far_points = 0;
    T v = _uv_ymin;
    for (int i = 0; i < _uv_roi_rows; ++i) {
        T u = _uv_xmin;
        for (int j = 0; j < _uv_roi_cols; ++j) {
            // point coordinate in image space (x, y)
            int id = i * _uv_roi_cols + j;

            // transform point from image space to ego-car ground space
            if (!project(u, v, &(_uv_2_xy[id]))) {
                _valid_xy[id] = false;
                ++count_fail_points;
                continue;
            }

            // ignore points which is too far away from origin
            if (_uv_2_xy[id].x() > max_distance || _uv_2_xy[id].y() > max_distance) {
                _valid_xy[id] = false;
                ++count_too_far_points;
                continue;
            }

            //
            if (_uv_2_xy[id].x() < 0) {
                _valid_xy[id] = false;
                ++count_too_far_points;
                continue;
            }

#if _DEBUG
            /*
             *     A    B    C
             *
             *     F    E    D
             */

            // point A
            if (i == 0 && j == 0) {
                XLOG(INFO) << "A: " << "(" << u << ", " << v << ") => ("
                           << _uv_2_xy[id](0) << ", " << _uv_2_xy[id](1) << ")";
            }
            // point B
            if (i == 0 && j == _uv_roi_cols / 2) {
                XLOG(INFO) << "B: " << "(" << u << ", " << v << ") => ("
                           << _uv_2_xy[id](0) << ", " << _uv_2_xy[id](1) << ")";
            }
            // point C
            if (i == 0 && j == _uv_roi_cols - 1) {
                XLOG(INFO) << "C: " << "(" << u << ", " << v << ") => ("
                           << _uv_2_xy[id](0) << ", " << _uv_2_xy[id](1) << ")";
            }
            // point D
            if (i == _uv_roi_rows - 1 && j == _uv_roi_cols - 1) {
                XLOG(INFO) << "D: " << "(" << u << ", " << v << ") => ("
                           << _uv_2_xy[id](0) << ", " << _uv_2_xy[id](1) << ")";
            }
            // point E
            if (i == _uv_roi_rows - 1 && j == _uv_roi_cols / 2) {
                XLOG(INFO) << "E: " << "(" << u << ", " << v << ") => ("
                           << _uv_2_xy[id](0) << ", " << _uv_2_xy[id](1) << ")";
            }
            // point F
            if (i == _uv_roi_rows - 1 && j == 0) {
                XLOG(INFO) << "F: " << "(" << u << ", " << v << ") => ("
                           << _uv_2_xy[id](0) << ", " << _uv_2_xy[id](1) << ")";
            }
#endif
            // update xy limits
            _xy_xmin = std::min(_xy_xmin, _uv_2_xy[id].x());
            _xy_ymin = std::min(_xy_ymin, _uv_2_xy[id].y());
            _xy_xmax = std::max(_xy_xmax, _uv_2_xy[id].x());
            _xy_ymax = std::max(_xy_ymax, _uv_2_xy[id].y());

            u += _uv_roi_x_step;
        }
        v += _uv_roi_y_step;
    }
    XLOG(INFO) << "#failed points = " << count_fail_points << " (" << _uv_roi_count << ").";
    XLOG(INFO) << "#too far points = " << count_too_far_points << " (" << _uv_roi_count << ").";
    XLOG(INFO) << " xy limits: "
               << "x_min=" << _xy_xmin << ", "
               << "x_max=" << _xy_xmax << ", "
               << "y_min=" << _xy_ymin << ", "
               << "y_max=" << _xy_ymax;

    _is_vis = visualize;
    if (_is_vis) {
        // build a 2D grid in ground plane and
        // find the index mapping for each valid point of image space
        _xy_image_xmin = std::min(std::floor(_xy_xmin), static_cast<T>(0));
        _xy_image_xmin = std::max(_xy_image_xmin, -max_distance);
        _xy_image_xmax = std::max(std::ceil(_xy_xmax), static_cast<T>(0));
        _xy_image_xmax = std::min(_xy_image_xmax, max_distance);
        _xy_image_ymin = std::min(std::floor(_xy_ymin), static_cast<T>(0));
        _xy_image_ymin = std::max(_xy_image_ymin, -max_distance);
        _xy_image_ymax = std::max(std::ceil(_xy_ymax), static_cast<T>(0));
        _xy_image_ymax = std::min(_xy_image_ymax, max_distance);

        _x_step = max_distance / static_cast<T>(1000);
        _y_step = max_distance / static_cast<T>(1000);

        _xy_image_cols = static_cast<int>(std::ceil((_xy_image_xmax - _xy_image_xmin) / _x_step)) + 1;
        _xy_image_rows = static_cast<int>(std::ceil((_xy_image_ymax - _xy_image_ymin) / _y_step)) + 1;
        XLOG(INFO) << "xy_image_xmin = " << _xy_image_xmin
                   << ", xy_image_xmax = " << _xy_image_xmax
                   << ", xy_image_ymin = " << _xy_image_ymin
                   << ", xy_image_ymax = " << _xy_image_ymax;
        XLOG(INFO) << "xy_image: step_x=" << _x_step << ", " << "step_y=" << _y_step;
        XLOG(INFO) << "xy_image: #cols = " << _xy_image_cols
                   << ", #rows = " << _xy_image_rows;

        _xy_grid.resize(_xy_image_rows * _xy_image_cols);
        T y = _xy_image_ymax;
        for (int i = 0; i < _xy_image_rows; i++) {
            T x = _xy_image_xmin;
            for (int j = 0; j < _xy_image_cols; j++) {
                _xy_grid[i * _xy_image_cols + j].x() = x;
                _xy_grid[i * _xy_image_cols + j].y() = y;
                x += _x_step;
            }
            y = y - _y_step;
        }

        //
        _uv_2_xy_image.resize(_uv_roi_count);
        T v = _uv_ymin;
        for (int i = 0; i < _uv_roi_rows; ++i) {
            T u = _uv_xmin;
            for (int j = 0; j < _uv_roi_cols; ++j) {
                int id = i * _uv_roi_cols + j;
                if (_valid_xy[id]) {
                    xy_2_xy_image_point(static_cast<T>(_uv_2_xy[id].x()),
                                        static_cast<T>(_uv_2_xy[id].y()),
                                        &_uv_2_xy_image[id]);
                } else {
                    _uv_2_xy_image[id].x = -1;
                    _uv_2_xy_image[id].y = -1;
                }
                u += _uv_roi_x_step;
            }
            v += _uv_roi_y_step;
        }
    }

    _is_init = true;

    XLOG(INFO) << "succ. to initialize projector.";
    return true;
}

template<typename T>
bool Projector<T>::xy_2_xy_image_point(const T &x, const T &y, cv::Point *p) const {
    if (!is_valid_xy_in_xy_image(x, y)) {
        return false;
    }

    int j = static_cast<int>(std::round((x - _xy_image_xmin) / _x_step));
    //CHECK_GE(j, 0);
    if (j < 0) {
        return false;
    }
    //CHECK_LT(j, _xy_image_cols);
    if (j >= _xy_image_cols) {
        return false;
    }

    int i = static_cast<int>(std::round((y - _xy_image_ymin) / _y_step));
    //CHECK_GE(i, 0);
    if (i < 0) {
        return false;
    }
    //CHECK_LT(i, _xy_image_rows);
    if (i >= _xy_image_rows) {
        return false;
    }

    i = (_xy_image_rows - 1) - i;

    p->x = j;
    p->y = i;

    return true;
}


template<typename T>
bool Projector<T>::is_valid_uv(const T &x, const T &y) const {
    if (!(x >= _uv_xmin && x <= _uv_xmax && y >= _uv_ymin && y <= _uv_ymax)) {
        XLOG(INFO) << "image point " << "(" << x << ", " << y << ")"
                   << " is not in ROI: "
                   << "x_min=" << _uv_xmin << ", "
                   << "x_max=" << _uv_xmax << ", "
                   << "y_min=" << _uv_ymin << ", "
                   << "y_max=" << _uv_ymax << ". ";
        return false;
    }
    return true;
}

template<typename T>
bool Projector<T>::uv_2_xy(const T &u, const T &v, Eigen::Matrix<T, 2, 1> *p) const {
    CHECK(p != nullptr);
    CHECK(_is_init) << "Error: projector is not initialized.";

    if (!is_valid_uv(u, v)) {
        return false;
    }

    int id = static_cast<int>(std::round((v - _uv_ymin) / _uv_roi_y_step)) * _uv_roi_cols
        + static_cast<int>(std::round((u - _uv_xmin) / _uv_roi_x_step));
    CHECK_GE(id, 0);
    CHECK_LT(id, _uv_roi_count);

    if (!_valid_xy.at(id)) {
        XLOG(INFO) << "image point " << "(" << u << ", " << v << ")"
                   << " is not valid for transformation.";
        return false;
    }

    p->x() = _uv_2_xy.at(id).x();
    p->y() = _uv_2_xy.at(id).y();
    return true;
}

template<typename T>
bool Projector<T>::uv_2_xy_image_point(const T &u, const T &v, cv::Point *p) const {
    CHECK(p != nullptr);
    CHECK(_is_init) << "Error: projector is not initialized.";

    if (!is_valid_uv(u, v)) {
        XLOG(INFO) << "image point " << "(" << u << ", " << v << ")"
                   << " is not in ROI.";
        return false;
    }

    int id = static_cast<int>(std::round((v - _uv_ymin) / _uv_roi_y_step)) * _uv_roi_cols
        + static_cast<int>(std::round((u - _uv_xmin) / _uv_roi_x_step));
    //CHECK_GE(id, 0);
    //CHECK_LT(id, _uv_roi_count);
    if (id < 0) {
        return false;
    }
    if (id >= _uv_roi_count) {
        return false;
    }

    if (!_valid_xy.at(id)) {
        XLOG(INFO) << "image point " << "(" << u << ", " << v << ")"
                   << " is not valid for transformation.";
        return false;
    }

    p->x = _uv_2_xy_image.at(id).x;
    p->y = _uv_2_xy_image.at(id).y;
    return true;
}

template<typename T>
bool Projector<T>::project(T u, T v, Eigen::Matrix<T, 2, 1>* xy_point) {
    if (xy_point == nullptr) {
        XLOG(ERROR) << "xy_point is a null pointer.";
        return false;
    }

    Eigen::Matrix<T, 3, 1> uv_point(u, v, static_cast<T>(1));
    Eigen::Matrix<T, 3, 1> xy_p = _trans_mat * uv_point;

#if _DEBUG
    XLOG(INFO) << "matirx: ";
    for (int i = 0; i < 3; ++i) {
        XLOG(INFO) << "              " << _trans_mat(i, 0) << ", "
                   << _trans_mat(i, 1) << ", "
                   << _trans_mat(i, 2) << "\n";
    }

    XLOG(INFO) << "uv_p: " << uv_point(0) << ", " << uv_point(1) << ", " << uv_point(2);
    XLOG(INFO) << "xy_p: " << xy_p(0) << ", " << xy_p(1) << ", " << xy_p(2);
#endif

    T scale = xy_p(2);
    if (std::abs(scale) < 1e-6) {
        XLOG(INFO) << "Cannot solve point: scale factor is too small (" << scale << "), "
                   << " u=" << u << " v=" << v << ".";
        return false;
    }

    (*xy_point) << xy_p(0) / scale, xy_p(1) / scale;
#if _DEBUG
    XLOG(INFO) << "*xy_point: " << (*xy_point)(0) << ", " << (*xy_point)(1);
#endif

    if (!std::isfinite((*xy_point)(0)) || std::isnan((*xy_point)(0)) ||
        !std::isfinite((*xy_point)(1)) || std::isnan((*xy_point)(1))) {
        XLOG(INFO) << "xy point is not valid: " << " u=" << u << " v=" << v << ".";
        return false;
    }

    return true;
}

}  // transformer_tool
}  // obstacle
}  // perception
}  // adu

#endif  // ADU_PERCEPTION_OBSTACLE_CAMERA_PARSER_TRANSFORMER_TOOL_PROJECTOR_HPP

