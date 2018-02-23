#ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_GEOMETRY_CAMERA_TRANSFORMER_H
#define ADU_PERCEPTION_OBSTACLE_CAMERA_GEOMETRY_CAMERA_TRANSFORMER_H

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "xlog.h"
#include "lib/config_manager/config_manager.h"

#include "obstacle/common/camera.h"
#include "obstacle/camera/common/util.h"
#include "obstacle/camera/interface/base_camera_transformer.h"

namespace adu {
namespace perception {
namespace obstacle {

class GeometryCameraTransformer : public BaseCameraTransformer {
public:

    GeometryCameraTransformer() : BaseCameraTransformer() {}

    virtual ~GeometryCameraTransformer() {}

    virtual bool init() override;

    virtual bool transform(const cv::Mat& frame, const CameraTransformerOptions& options,
                           std::vector<VisualObjectPtr>* objects) override;

    bool instance_transform(const double &h, const double &w, const double &l, const double &alpha_deg,
                            const Eigen::Vector2d &upper_left, const Eigen::Vector2d &lower_right,
                            double &distance_w, double &distance_h,
                            Eigen::Vector2d &mass_center_pixel);

    void set_visualization(bool flag);

    virtual std::string name() const override;

private:

    bool load_camera_intrinsics(const std::string &file_name);

    void rotate_object(double alpha_deg, std::vector<Eigen::Vector3d > &corners) const;

    double distance_binary_search(const int &target_pixel_length, const bool &use_width,
                                  const Eigen::Matrix<double, 3, 1> &mass_center_v) const;

    void mass_center_search(const Eigen::Matrix<double, 2, 1> &target_box_center_pixel,
                            const double &curr_d, Eigen::Matrix<double, 3, 1> &mass_center_v,
                            Eigen::Matrix<double, 2, 1> &mass_center_pixel) const;

    Eigen::Matrix<double, 3, 1> to_unit_v(const Eigen::Matrix<double, 3, 1> &v) const;

    Camera<double> _camera_model;
    std::vector<Eigen::Vector3d> _corners;
    static const int max_distance_binary_search_depth = 20;
    static const int max_mass_center_search_depth = 10;
    bool _visualization = false;
};

} //namespace obstacle
} //namespace perception
} //namespace adu

#endif  // ADU_PERCEPTION_OBSTACLE_CAMERA_GEOMETRY_CAMERA_TRANSFORMER_H
