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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CAMERA_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CAMERA_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace apollo {
namespace perception {

template <typename T> class Camera;
template <typename T> class CameraDistort;

/**@brief Print the matrix.*/
template <typename T>
std::ostream& operator<< (std::ostream& cout, const Camera<T>& camera);

template <typename T>
std::ostream& operator<< (std::ostream& cout, const CameraDistort<T>& camera);

/**@brief camera intrinsic of pin-hole camera model*/
template <typename T>
class Camera {
public:
    Camera() {
        _focal_length_x = 1;
        _focal_length_y = 1;
        _center_x = 0;
        _center_y = 0;
        _intrinsic(0, 0) = 1;
        _intrinsic(0, 1) = 0;
        _intrinsic(0, 2) = 0;
        _intrinsic(1, 0) = 0;
        _intrinsic(1, 1) = 1;
        _intrinsic(1, 2) = 0;
        _intrinsic(2, 0) = 0;
        _intrinsic(2, 1) = 0;
        _intrinsic(2, 2) = 1;
        _width = 1;
        _height = 1;
    }

    void set(const Eigen::Matrix<T, 3, 3>& params, T w, T h) {
        _intrinsic = params;
        _focal_length_x = _intrinsic(0, 0);
        _focal_length_y = _intrinsic(1, 1);
        _center_x = _intrinsic(0, 2);
        _center_y = _intrinsic(1, 2);
        _width = w;
        _height = h;
    }

    void set(T focal_length_x, T focal_length_y, T center_x, T center_y, T w, T h) {
        _focal_length_x = focal_length_x;
        _focal_length_y = focal_length_y;
        _center_x = center_x;
        _center_y = center_y;
        _width = w;
        _height = h;
        _intrinsic(0, 0) = _focal_length_x;
        _intrinsic(1, 1) = _focal_length_y;
        _intrinsic(0, 2) = _center_x;
        _intrinsic(1, 2) = _center_y;
    }

    /**@brief Project a 3D point on an image. */
    virtual Eigen::Matrix<T, 2, 1> project(const Eigen::Matrix<T, 3, 1>& pt3d) const {

        Eigen::Matrix<T, 2, 1> pt2d;

        pt2d[0] = pt3d[0] / pt3d[2];
        pt2d[1] = pt3d[1] / pt3d[2];

        return pixel_denormalize(pt2d);
    }

    /**@brief Unproject a pixel to 3D point on a given XY plane, where z = 1 */
    virtual Eigen::Matrix<T, 3, 1> unproject(const Eigen::Matrix<T, 2, 1>& pt2d) const {

        Eigen::Matrix<T, 3, 1> pt3d;

        Eigen::Matrix<T, 2, 1> pt2d_tmp = pixel_normalize(pt2d);

        pt3d[0] = pt2d_tmp[0];
        pt3d[1] = pt2d_tmp[1];
        pt3d[2] = 1;

        return pt3d;
    }

    /**@brief Project a 3D point on an image. */
    virtual Eigen::Matrix<T, 2, 1> project(const Eigen::Matrix<T, 4, 4>& transform,
        const Eigen::Matrix<T, 3, 1>& pt3d) const {

        Eigen::Matrix<T, 3, 1> local_pt3d;
        local_pt3d[0] = transform(0, 0) * pt3d[0] +
            transform(0, 1) * pt3d[1] + transform(0, 2) * pt3d[2] + transform(0, 3);
        local_pt3d[1] = transform(1, 0) * pt3d[0] +
            transform(1, 1) * pt3d[1] + transform(1, 2) * pt3d[2] + transform(1, 3);
        local_pt3d[2] = transform(2, 0) * pt3d[0] +
            transform(2, 1) * pt3d[1] + transform(2, 2) * pt3d[2] + transform(2, 3);

        return project(local_pt3d);
    }

    /**@brief Check if a 3D point is in the view*/
    bool is_in_view(const Eigen::Matrix<T, 4, 4>& transform,
        const Eigen::Matrix<T, 3, 1>& pt3d) const {
        Eigen::Matrix<T, 3, 1> local_pt3d;
        local_pt3d[0] = transform(0, 0) * pt3d[0] +
            transform(0, 1) * pt3d[1] + transform(0, 2) * pt3d[2] + transform(0, 3);
        local_pt3d[1] = transform(1, 0) * pt3d[0] +
            transform(1, 1) * pt3d[1] + transform(1, 2) * pt3d[2] + transform(1, 3);
        local_pt3d[2] = transform(2, 0) * pt3d[0] +
            transform(2, 1) * pt3d[1] + transform(2, 2) * pt3d[2] + transform(2, 3);
        if (local_pt3d[2] <= 0) {
            return false;
        }

        Eigen::Matrix<T, 2, 1> pt2d = project(local_pt3d);
        if (pt2d[0] > 0 && pt2d[0] < _width && pt2d[1] > 0 && pt2d[1] < _height) {
            return true;
        }
        return false;
    }

    /**@brief Get the x focal length. */
    inline T get_focal_length_x() const {
        return _focal_length_x;
    }
    /**@brief Get the y focal length. */
    inline T get_focal_length_y() const {
        return _focal_length_y;
    }
    /**@brief Get the optical center x. */
    inline T get_center_x() const {
        return _center_x;
    }
    /**@brief Get the optical center y. */
    inline T get_center_y() const {
        return _center_y;
    }
    /**@brief Get the intrinsic matrix K. */
    inline const Eigen::Matrix<T, 3, 3>& get_intrinsic() const {
        return _intrinsic;
    }
    /**@brief Get the intrinsic matrix K. */
    inline Eigen::Matrix<T, 3, 3>& get_intrinsic() {
        return _intrinsic;
    }
    /**@brief Get the image width */
    inline T get_width() const {
        return _width;
    }
    /**@brief Get the image height */
    inline T get_height() const {
        return _height;
    }

    friend std::ostream& operator << <> (std::ostream& out, const Camera<T>& camera);

protected:
    /**@brief Normalize a 2D pixel. Convert a 2D pixel as if the image is taken with a camera,
     * whose K = identity matrix. */
    virtual Eigen::Matrix<T, 2, 1> pixel_normalize(
        const Eigen::Matrix<T, 2, 1>& pt2d) const {

        Eigen::Matrix<T, 2, 1> p;
        p[0] = (pt2d[0] - _center_x) / ((double)_focal_length_x);
        p[1] = (pt2d[1] - _center_y) / ((double)_focal_length_y);

        return p;
    }

    /**@brief Denormalize a 2D pixel. Convert a 2D pixel as if the image is taken with a camera,
     * whose K = _intrinsic. */
    virtual Eigen::Matrix<T, 2, 1> pixel_denormalize(
        const Eigen::Matrix<T, 2, 1>& pt2d) const {

        Eigen::Matrix<T, 2, 1> p;
        p[0] = pt2d[0] * _focal_length_x + _center_x;
        p[1] = pt2d[1] * _focal_length_y + _center_y;

        return p;
    }

protected:
    /**@brief The camera intrinsic matrix. */
    Eigen::Matrix<T, 3, 3> _intrinsic;
    /**@brief The focal length x. */
    T _focal_length_x;
    /**@brief The focal length y. */
    T _focal_length_y;
    /**@brief The optical center x. */
    T _center_x;
    /**@brief The optical center y. */
    T _center_y;
    /**@brief Image width */
    T _width;
    /**@brief Image height */
    T _height;
};

/**@brief camera intrinsic of pin-hole camera model with distortion*/
template <typename T>
class CameraDistort : public Camera<T> {
public:
    /**@brief The default constructor. */
    CameraDistort() {
        _distort_params[0] = 0;
        _distort_params[1] = 0;
        _distort_params[2] = 0;
        _distort_params[3] = 0;
        _distort_params[4] = 0;
    }

    /**@brief Project a 3D point on an image. */
    virtual Eigen::Matrix<T, 2, 1> project(const Eigen::Matrix<T, 3, 1>& pt3d) const {
        Eigen::Matrix<T, 2, 1> pt2d;
        pt2d[0] = pt3d[0] / pt3d[2];
        pt2d[1] = pt3d[1] / pt3d[2];
        return pixel_denormalize(pt2d);
    }

    /**@brief Unproject a pixel to 3D point on a given XY plane, where z = 1 */
    virtual Eigen::Matrix<T, 3, 1> unproject(const Eigen::Matrix<T, 2, 1>& pt2d) const {

        Eigen::Matrix<T, 3, 1> pt3d;

        Eigen::Matrix<T, 2, 1> pt2d_tmp = pixel_normalize(pt2d);

        pt3d[0] = pt2d_tmp[0];
        pt3d[1] = pt2d_tmp[1];
        pt3d[2] = 1;

        return pt3d;
    }

    /**@brief Project a 3D point on an image. */
    virtual Eigen::Matrix<T, 2, 1> project(const Eigen::Matrix<T, 4, 4>& transform,
        const Eigen::Matrix<T, 3, 1>& pt3d) const {

        Eigen::Matrix<T, 3, 1> local_pt3d;
        local_pt3d[0] = transform(0, 0) * pt3d[0] +
            transform(0, 1) * pt3d[1] + transform(0, 2) * pt3d[2] + transform(0, 3);
        local_pt3d[1] = transform(1, 0) * pt3d[0] +
            transform(1, 1) * pt3d[1] + transform(1, 2) * pt3d[2] + transform(1, 3);
        local_pt3d[2] = transform(2, 0) * pt3d[0] +
            transform(2, 1) * pt3d[1] + transform(2, 2) * pt3d[2] + transform(2, 3);

        return project(local_pt3d);
    }

    /**@brief Set the distortion parameters. */
    void set_distort_params(T d0, T d1, T d2, T d3, T d4) {
        _distort_params[0] = d0;
        _distort_params[0] = d1;
        _distort_params[0] = d2;
        _distort_params[0] = d3;
        _distort_params[0] = d4;
    }

    /**@brief Set the distortion parameters. */
    inline void set_distort_params(const Eigen::Matrix<T, 5, 1>& params) {
        _distort_params = params;
    }

    /**@brief Get the distortion parameters. */
    inline const Eigen::Matrix<T, 5, 1>& get_distort_params() const {
        return _distort_params;
    }

    /**@brief Get the distortion parameters. */
    inline Eigen::Matrix<T, 5, 1>& get_distort_params() {
        return _distort_params;
    }

    friend std::ostream& operator << <> (std::ostream& out,
        const CameraDistort<T>& camera);

protected:
    /**@brief Normalize a 2D pixel. Convert a 2D pixel as if the image is taken with a camera,
     * whose K = identity matrix. */
    virtual Eigen::Matrix<T, 2, 1> pixel_normalize(const Eigen::Matrix<T, 2, 1>& pt2d) const {

        Eigen::Matrix<T, 2, 1> pt2d_distort = Camera<T>::pixel_normalize(pt2d);

        Eigen::Matrix<T, 2, 1> pt2d_undistort = pt2d_distort;   // Initial guess
        for (unsigned int i = 0; i < 20; ++i) {
            T r_sq = pt2d_undistort[0] * pt2d_undistort[0] +
                 pt2d_undistort[1] * pt2d_undistort[1];
            T k_radial =  1.0 + _distort_params[0] * r_sq + _distort_params[1] *
                    r_sq * r_sq + _distort_params[4] * r_sq * r_sq * r_sq;
            T delta_x_0 = 2 * _distort_params[2] * pt2d_undistort[0] * pt2d_undistort[1] +
                    _distort_params[3] * (r_sq + 2 * pt2d_undistort[0] * pt2d_undistort[0]);
            T delta_x_1 = _distort_params[2] * (r_sq + 2*pt2d_undistort[1]*pt2d_undistort[1]) +
                    2 * _distort_params[3] * pt2d_undistort[0] * pt2d_undistort[1];
            pt2d_undistort[0] = (pt2d_distort[0] - delta_x_0) / k_radial;
            pt2d_undistort[1] = (pt2d_distort[1] - delta_x_1) / k_radial;
        }
        return pt2d_undistort;
    }

    /**@brief Denormalize a 2D pixel. Convert a 2D pixel as if the image is taken with a camera,
     * whose K = _intrinsic. */
    virtual Eigen::Matrix<T, 2, 1> pixel_denormalize(
        const Eigen::Matrix<T, 2, 1>& pt2d) const {
        // Add distortion
        T r_sq = pt2d[0]*pt2d[0] + pt2d[1]*pt2d[1];
        Eigen::Matrix<T, 2, 1> pt2d_radial = pt2d * (1 + _distort_params[0]*r_sq +
                _distort_params[1]*r_sq*r_sq + _distort_params[4]*r_sq*r_sq*r_sq);
        Eigen::Matrix<T, 2, 1> dpt2d;
        dpt2d[0] = 2*_distort_params[2]*pt2d[0]*pt2d[1] +
            _distort_params[3]*(r_sq + 2*pt2d[0]*pt2d[0]);
        dpt2d[1] = _distort_params[2]*(r_sq + 2*pt2d[1]*pt2d[1]) +
            2*_distort_params[3]*pt2d[0]*pt2d[1];

        Eigen::Matrix<T, 2, 1> pt2d_undistort;
        pt2d_undistort[0] = pt2d_radial[0] + dpt2d[0];
        pt2d_undistort[1] = pt2d_radial[1] + dpt2d[1];
        // Add intrinsic K
        return Camera<T>::pixel_denormalize(pt2d_undistort);
    }

protected:
    /**@brief The distortion parameters.
     *
     * See here for the definition of the parameters:
     * http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
     */
    Eigen::Matrix<T, 5, 1> _distort_params;
};

template <typename T>
std::ostream& operator<< (std::ostream& cout, const Camera<T>& camera) {
    cout << camera._intrinsic << "\n ["
        << camera._width << "," << camera._height << "]\n";
    return cout;
}

template <typename T>
std::ostream& operator<< (std::ostream& cout, const CameraDistort<T>& camera) {
    cout << camera._intrinsic << "\n ["
        << camera._width << "," << camera._height << "]\n";
    cout << camera._distort_params;

    return cout;
}

}  // namespace perception
}  // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CAMERA_H_
