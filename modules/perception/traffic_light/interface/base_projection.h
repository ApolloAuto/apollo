// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author zhengwenchao(zhengwenchao@baidu.com)
// @date 2016/09/08 17:32:56
// @file: base_projection.h
// @brief: the interface for mark a traffic light region on the image.
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PROJECTION_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PROJECTION_H

#include "modules/perception/lib/base/noncopyable.h"
#include "modules/perception/lib/base/registerer.h"

#include "modules/perception/traffic_light/base/pose.h"
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {

DECLARE_string(traffic_light_projection);

struct ProjectOption {
  explicit ProjectOption(const CameraId &id) : camera_id(id) {

  }

  CameraId camera_id;
};

//@brief The Basic Camera Coeffient
struct CameraCoeffient {
 public:
  bool init(const std::string &camera_type,
            const std::string &lidar2gps_matrix_file_name,
            const std::string &lidar2camera_matrix_file_name,
            const std::string &camera_intrinsic_matrix_file);
 private:
  bool init_lidar_to_gps_matrix(const std::string &matrix_file);
  bool init_lidar_to_camera_matrix(const std::string &matrix_file);
  bool init_camera_intrinsic_matrix_and_distort_params(
      const std::string &camera_intrinsic_file);

 public:
  std::string camera_type_str;
  Eigen::Matrix4d lidar2gps;    //Lidar->GPS Matrix
  Eigen::Matrix4d lidar2camera; //Lidar->Camera Matrix
  Eigen::Matrix4d gps2camera; //Lidar->Camera Matrix
  Eigen::Matrix<double, 3, 4> camera_intrinsic; // Camera Intrinsic
  Eigen::Matrix<double, 5, 1> distort_params;  // distortion parameters
  size_t image_height;
  size_t image_width;
};

//@brief Projection project traffic light(3D) to a region on the image(2D).
//       The data(map & params) for projection should be load in init function.
class BaseProjection {
 public:
  BaseProjection() = default;

  virtual ~BaseProjection() = default;

  //@brief project the traffic_light to a region on the image.
  //@params [in] camera's coeffients
  //@params [in] car's pose
  //@params [in] option for project
  //@params [in/out] in:traffic light 's x,y,z
  //                 out:map info & the region on the image.
  virtual bool project(const CameraCoeffient &camera_coeffient,
                       const Eigen::Matrix4d &pose,
                       const apollo::hdmap::Signal &tl_info,
                       Light *light) const = 0;
};

REGISTER_REGISTERER(BaseProjection);
#define REGISTER_PROJECTION(name) REGISTER_CLASS(BaseProjection, name)

template<typename T>
T arithmetic_mean(const std::vector<T> &data) {
  assert(data.size() > 0);

  T sum = std::accumulate(data.begin(), data.end(), T());
  return sum / data.size();
}

template<typename T>
T square(const T &d) {
  return d * d;
}

template<typename T>
T degree_to_radians(const T &angle) {
  return angle * M_PI / 180;
}

//@brief load transformation_matrix from file
bool load_transformation_matrix_from_file(const std::string &file_name,
                                          Eigen::Matrix4d *transformation_matrix);

/*@brief load Matrix4d from file
  in File
  key:
    0:
      0: XX
      1: YY
      ...
    ...
*/
bool load_matrix4d_from_file(const std::string &file_name, const std::string &key,
                             Eigen::Matrix4d *matrix);
} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PROJECTION_H

