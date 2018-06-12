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

#include <algorithm>

namespace apollo {
namespace perception {

template <typename T>
class CameraModel;
template <typename T>
class CameraDistort;

/**@brief Print the matrix.*/
template <typename T>
std::ostream& operator<<(std::ostream& cout, const CameraModel<T>& camera);

template <typename T>
std::ostream& operator<<(std::ostream& cout, const CameraDistort<T>& camera);

/**@brief camera intrinsic of pin-hole camera model*/
template <typename T>
class CameraModel {
 public:
  CameraModel() {
    focal_length_x_ = 1;
    focal_length_y_ = 1;
    center_x_ = 0;
    center_y_ = 0;
    intrinsic_(0, 0) = 1;
    intrinsic_(0, 1) = 0;
    intrinsic_(0, 2) = 0;
    intrinsic_(1, 0) = 0;
    intrinsic_(1, 1) = 1;
    intrinsic_(1, 2) = 0;
    intrinsic_(2, 0) = 0;
    intrinsic_(2, 1) = 0;
    intrinsic_(2, 2) = 1;
    width_ = 1;
    height_ = 1;
  }

  void set(const Eigen::Matrix<T, 3, 3>& params, T w, T h) {
    intrinsic_ = params;
    focal_length_x_ = intrinsic_(0, 0);
    focal_length_y_ = intrinsic_(1, 1);
    center_x_ = intrinsic_(0, 2);
    center_y_ = intrinsic_(1, 2);
    width_ = w;
    height_ = h;
  }

  void set(T focal_length_x, T focal_length_y, T center_x, T center_y, T w,
           T h) {
    focal_length_x_ = focal_length_x;
    focal_length_y_ = focal_length_y;
    center_x_ = center_x;
    center_y_ = center_y;
    width_ = w;
    height_ = h;
    intrinsic_(0, 0) = focal_length_x_;
    intrinsic_(1, 1) = focal_length_y_;
    intrinsic_(0, 2) = center_x_;
    intrinsic_(1, 2) = center_y_;
  }

  /**@brief Project a 3D point on an image. */
  virtual Eigen::Matrix<T, 2, 1> project(
      const Eigen::Matrix<T, 3, 1>& pt3d) const {
    Eigen::Matrix<T, 2, 1> pt2d;

    pt2d[0] = pt3d[0] / pt3d[2];
    pt2d[1] = pt3d[1] / pt3d[2];

    return pixel_denormalize(pt2d);
  }

  /**@brief Unproject a pixel to 3D point on a given XY plane, where z = 1 */
  virtual Eigen::Matrix<T, 3, 1> unproject(
      const Eigen::Matrix<T, 2, 1>& pt2d) const {
    Eigen::Matrix<T, 3, 1> pt3d;

    Eigen::Matrix<T, 2, 1> pt2d_tmp = pixel_normalize(pt2d);

    pt3d[0] = pt2d_tmp[0];
    pt3d[1] = pt2d_tmp[1];
    pt3d[2] = 1;

    return pt3d;
  }

  /**@brief Project a 3D point on an image. */
  virtual Eigen::Matrix<T, 2, 1> project(
      const Eigen::Matrix<T, 4, 4>& transform,
      const Eigen::Matrix<T, 3, 1>& pt3d) const {
    Eigen::Matrix<T, 3, 1> local_pt3d;
    local_pt3d[0] = transform(0, 0) * pt3d[0] + transform(0, 1) * pt3d[1] +
                    transform(0, 2) * pt3d[2] + transform(0, 3);
    local_pt3d[1] = transform(1, 0) * pt3d[0] + transform(1, 1) * pt3d[1] +
                    transform(1, 2) * pt3d[2] + transform(1, 3);
    local_pt3d[2] = transform(2, 0) * pt3d[0] + transform(2, 1) * pt3d[1] +
                    transform(2, 2) * pt3d[2] + transform(2, 3);

    return project(local_pt3d);
  }

  /**@brief Check if a 3D point is in the view*/
  bool is_in_view(const Eigen::Matrix<T, 4, 4>& transform,
                  const Eigen::Matrix<T, 3, 1>& pt3d) const {
    Eigen::Matrix<T, 3, 1> local_pt3d;
    local_pt3d[0] = transform(0, 0) * pt3d[0] + transform(0, 1) * pt3d[1] +
                    transform(0, 2) * pt3d[2] + transform(0, 3);
    local_pt3d[1] = transform(1, 0) * pt3d[0] + transform(1, 1) * pt3d[1] +
                    transform(1, 2) * pt3d[2] + transform(1, 3);
    local_pt3d[2] = transform(2, 0) * pt3d[0] + transform(2, 1) * pt3d[1] +
                    transform(2, 2) * pt3d[2] + transform(2, 3);
    if (local_pt3d[2] <= 0) {
      return false;
    }

    Eigen::Matrix<T, 2, 1> pt2d = project(local_pt3d);
    if (pt2d[0] > 0 && pt2d[0] < width_ && pt2d[1] > 0 && pt2d[1] < height_) {
      return true;
    }
    return false;
  }

  /**@brief Get the x focal length. */
  inline T get_focal_length_x() const { return focal_length_x_; }
  /**@brief Get the y focal length. */
  inline T get_focal_length_y() const { return focal_length_y_; }
  /**@brief Get the optical center x. */
  inline T get_center_x() const { return center_x_; }
  /**@brief Get the optical center y. */
  inline T get_center_y() const { return center_y_; }
  /**@brief Get the intrinsic matrix K. */
  inline const Eigen::Matrix<T, 3, 3>& get_intrinsic() const {
    return intrinsic_;
  }
  /**@brief Get the intrinsic matrix K. */
  inline Eigen::Matrix<T, 3, 3>& get_intrinsic() { return intrinsic_; }
  /**@brief Get the image width */
  inline T get_width() const { return width_; }
  /**@brief Get the image height */
  inline T get_height() const { return height_; }

  friend std::ostream& operator<<<>(std::ostream& out,
                                    const CameraModel<T>& camera);

 protected:
  /**@brief Normalize a 2D pixel. Convert a 2D pixel as if the image is taken
   * with a camera,
   * whose K = identity matrix. */
  virtual Eigen::Matrix<T, 2, 1> pixel_normalize(
      const Eigen::Matrix<T, 2, 1>& pt2d) const {
    Eigen::Matrix<T, 2, 1> p;
    p[0] = (pt2d[0] - center_x_) / focal_length_x_;
    p[1] = (pt2d[1] - center_y_) / focal_length_y_;

    return p;
  }

  /**@brief Denormalize a 2D pixel. Convert a 2D pixel as if the image is taken
   * with a camera,
   * whose K = intrinsic_. */
  virtual Eigen::Matrix<T, 2, 1> pixel_denormalize(
      const Eigen::Matrix<T, 2, 1>& pt2d) const {
    Eigen::Matrix<T, 2, 1> p;
    p[0] = pt2d[0] * focal_length_x_ + center_x_;
    p[1] = pt2d[1] * focal_length_y_ + center_y_;

    return p;
  }

 protected:
  /**@brief The camera intrinsic matrix. */
  Eigen::Matrix<T, 3, 3> intrinsic_;
  /**@brief The focal length x. */
  T focal_length_x_;
  /**@brief The focal length y. */
  T focal_length_y_;
  /**@brief The optical center x. */
  T center_x_;
  /**@brief The optical center y. */
  T center_y_;
  /**@brief Image width */
  T width_;
  /**@brief Image height */
  T height_;
};

/**@brief camera intrinsic of pin-hole camera model with distortion*/
template <typename T>
class CameraDistort : public CameraModel<T> {
 public:
  /**@brief The default constructor. */
  CameraDistort() {
    distort_params_[0] = 0;
    distort_params_[1] = 0;
    distort_params_[2] = 0;
    distort_params_[3] = 0;
    distort_params_[4] = 0;
  }

  /**@brief Project a 3D point on an image. */
  virtual Eigen::Matrix<T, 2, 1> project(
      const Eigen::Matrix<T, 3, 1>& pt3d) const {
    Eigen::Matrix<T, 2, 1> pt2d;
    pt2d[0] = pt3d[0] / pt3d[2];
    pt2d[1] = pt3d[1] / pt3d[2];
    return pixel_denormalize(pt2d);
  }

  /**@brief Unproject a pixel to 3D point on a given XY plane, where z = 1 */
  virtual Eigen::Matrix<T, 3, 1> unproject(
      const Eigen::Matrix<T, 2, 1>& pt2d) const {
    Eigen::Matrix<T, 3, 1> pt3d;

    Eigen::Matrix<T, 2, 1> pt2d_tmp = pixel_normalize(pt2d);

    pt3d[0] = pt2d_tmp[0];
    pt3d[1] = pt2d_tmp[1];
    pt3d[2] = 1;

    return pt3d;
  }

  /**@brief Project a 3D point on an image. */
  virtual Eigen::Matrix<T, 2, 1> project(
      const Eigen::Matrix<T, 4, 4>& transform,
      const Eigen::Matrix<T, 3, 1>& pt3d) const {
    Eigen::Matrix<T, 3, 1> local_pt3d;
    local_pt3d[0] = transform(0, 0) * pt3d[0] + transform(0, 1) * pt3d[1] +
                    transform(0, 2) * pt3d[2] + transform(0, 3);
    local_pt3d[1] = transform(1, 0) * pt3d[0] + transform(1, 1) * pt3d[1] +
                    transform(1, 2) * pt3d[2] + transform(1, 3);
    local_pt3d[2] = transform(2, 0) * pt3d[0] + transform(2, 1) * pt3d[1] +
                    transform(2, 2) * pt3d[2] + transform(2, 3);

    return project(local_pt3d);
  }

  /**@brief Set the distortion parameters. */
  void set_distort_params(T d0, T d1, T d2, T d3, T d4) {
    distort_params_[0] = d0;
    distort_params_[0] = d1;
    distort_params_[0] = d2;
    distort_params_[0] = d3;
    distort_params_[0] = d4;
  }

  /**@brief Set the distortion parameters. */
  inline void set_distort_params(const Eigen::Matrix<T, 5, 1>& params) {
    distort_params_ = params;
  }

  /**@brief Get the distortion parameters. */
  inline const Eigen::Matrix<T, 5, 1>& get_distort_params() const {
    return distort_params_;
  }

  /**@brief Get the distortion parameters. */
  inline Eigen::Matrix<T, 5, 1>& get_distort_params() {
    return distort_params_;
  }

  friend std::ostream& operator<<<>(std::ostream& out,
                                    const CameraDistort<T>& camera);

 protected:
  /**@brief Normalize a 2D pixel. Convert a 2D pixel as if the image is taken
   * with a camera,
   * whose K = identity matrix. */
  virtual Eigen::Matrix<T, 2, 1> pixel_normalize(
      const Eigen::Matrix<T, 2, 1>& pt2d) const {
    Eigen::Matrix<T, 2, 1> pt2d_distort = CameraModel<T>::pixel_normalize(pt2d);

    Eigen::Matrix<T, 2, 1> pt2d_undistort = pt2d_distort;  // Initial guess
    for (unsigned int i = 0; i < 20; ++i) {
      T r_sq = pt2d_undistort[0] * pt2d_undistort[0] +
               pt2d_undistort[1] * pt2d_undistort[1];
      T k_radial = 1.0 + distort_params_[0] * r_sq +
                   distort_params_[1] * r_sq * r_sq +
                   distort_params_[4] * r_sq * r_sq * r_sq;
      T delta_x_0 =
          2 * distort_params_[2] * pt2d_undistort[0] * pt2d_undistort[1] +
          distort_params_[3] *
              (r_sq + 2 * pt2d_undistort[0] * pt2d_undistort[0]);
      T delta_x_1 =
          distort_params_[2] *
              (r_sq + 2 * pt2d_undistort[1] * pt2d_undistort[1]) +
          2 * distort_params_[3] * pt2d_undistort[0] * pt2d_undistort[1];
      pt2d_undistort[0] = (pt2d_distort[0] - delta_x_0) / k_radial;
      pt2d_undistort[1] = (pt2d_distort[1] - delta_x_1) / k_radial;
    }
    return pt2d_undistort;
  }

  /**@brief Denormalize a 2D pixel. Convert a 2D pixel as if the image is taken
   * with a camera,
   * whose K = intrinsic_. */
  virtual Eigen::Matrix<T, 2, 1> pixel_denormalize(
      const Eigen::Matrix<T, 2, 1>& pt2d) const {
    // Add distortion
    T r_sq = pt2d[0] * pt2d[0] + pt2d[1] * pt2d[1];
    Eigen::Matrix<T, 2, 1> pt2d_radial =
        pt2d *
        (1 + distort_params_[0] * r_sq + distort_params_[1] * r_sq * r_sq +
         distort_params_[4] * r_sq * r_sq * r_sq);
    Eigen::Matrix<T, 2, 1> dpt2d;
    dpt2d[0] = 2 * distort_params_[2] * pt2d[0] * pt2d[1] +
               distort_params_[3] * (r_sq + 2 * pt2d[0] * pt2d[0]);
    dpt2d[1] = distort_params_[2] * (r_sq + 2 * pt2d[1] * pt2d[1]) +
               2 * distort_params_[3] * pt2d[0] * pt2d[1];

    Eigen::Matrix<T, 2, 1> pt2d_undistort;
    pt2d_undistort[0] = pt2d_radial[0] + dpt2d[0];
    pt2d_undistort[1] = pt2d_radial[1] + dpt2d[1];
    // Add intrinsic K
    return CameraModel<T>::pixel_denormalize(pt2d_undistort);
  }

 protected:
  /**@brief The distortion parameters.
   *
   * See here for the definition of the parameters:
   * http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
   */
  Eigen::Matrix<T, 5, 1> distort_params_;
};

template <typename T>
std::ostream& operator<<(std::ostream& cout, const CameraModel<T>& camera) {
  cout << camera.intrinsic_ << "\n [" << camera.width_ << "," << camera.height_
       << "]\n";
  return cout;
}

template <typename T>
std::ostream& operator<<(std::ostream& cout, const CameraDistort<T>& camera) {
  cout << camera.intrinsic_ << "\n [" << camera.width_ << "," << camera.height_
       << "]\n";
  cout << camera.distort_params_;

  return cout;
}

typedef CameraModel<double> CameraD;
typedef CameraDistort<double> CameraDistortD;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CAMERA_H_
