
#include "geometry_camera_transformer.h"

namespace adu {
namespace perception {
namespace obstacle {

bool GeometryCameraTransformer::init() {

    if (!load_camera_intrinsics(FLAGS_onsemi_obstacle_intrinsics)) {
        XLOG(ERROR) << "camera intrinsics not found: " << FLAGS_onsemi_obstacle_intrinsics;
        return false;
    }

    return true;
}

bool GeometryCameraTransformer::transform(const cv::Mat& frame,
                                          const CameraTransformerOptions& options,
                                          std::vector<VisualObjectPtr>* objects) {
    if (!objects) { return false; }

    for (auto &obj: *objects) {
        // TODO Physical Size sanity check based on type
        double deg_alpha = obj-> alpha * 180.0 / M_PI;

        Eigen::Vector2d upper_left(obj->upper_left[0], obj->upper_left[1]);
        Eigen::Vector2d lower_right(obj->lower_right[0], obj->lower_right[1]);
        double distance_w = 0.0;
        double distance_h = 0.0;
        Eigen::Vector2d mass_center_pixel;
        instance_transform(obj->height, obj->width, obj->length,
                           deg_alpha, upper_left, lower_right,
                           distance_w, distance_h,
                           mass_center_pixel);

        // TODO Choose distance_h or distance_w, considering truncation, type, longer side, or strategy
        obj->distance = distance_h; // distance_h, distance_w
        Eigen::Vector3d camera_ray = _camera_model.unproject(mass_center_pixel);

        // Angles
        // This definition is correct for OpenGL viewer...
        // double beta = std::atan2(camera_ray.x(), camera_ray.z());

        // This definition is correct for benchmark...
        double beta = std::atan2(camera_ray.z(), camera_ray.x());
        double theta = obj->alpha + beta;
        if (theta > M_PI) {
            theta -= 2*M_PI;
        }
        else if (theta < -M_PI) {
            theta += 2*M_PI;
        }
        obj->theta = theta;

        // Center (3D Mass Center of 3D BBox)
        double scale = obj->distance / sqrt(camera_ray.x() * camera_ray.x()
                                            + camera_ray.y() * camera_ray.y()
                                            + camera_ray.z() * camera_ray.z());
        obj->center = Eigen::Vector3d(camera_ray.x() * scale,
                                      camera_ray.y() * scale,
                                      camera_ray.z() * scale);

//        if (_visualization) {
//            obj->pts8.clear();
//            for (size_t i = 0; i < _corners.size(); ++i) {
//                Eigen::Vector2d point_2d = _camera_model.project(_corners[i] + obj->center);
//                obj->pts8.push_back(static_cast<float>(point_2d.x()));
//                obj->pts8.push_back(static_cast<float>(point_2d.y()));
//            }
//
//            // 3D center projection
//            Eigen::Vector2d point_2d = _camera_model.project(obj->center);
//            obj->pts8.push_back(static_cast<float>(point_2d.x()));
//            obj->pts8.push_back(static_cast<float>(point_2d.y()));
//        }
    }

    return true;
}

bool GeometryCameraTransformer::instance_transform(const double &h, const double &w, const double &l,
                                                   const double &alpha_deg,
                                                   const Eigen::Vector2d &upper_left,
                                                   const Eigen::Vector2d &lower_right,
                                                   double &distance_w, double &distance_h,
                                                   Eigen::Vector2d &mass_center_pixel) {
    // Target Goals: Projection target
    int pixel_width = static_cast<int>(lower_right.x() - upper_left.x());
    int pixel_height = static_cast<int>(lower_right.y() - upper_left.y());

    // Target Goals: Box center pixel
    Eigen::Matrix<double, 2, 1> target_box_center_pixel;
    target_box_center_pixel.x() = (lower_right.x() + upper_left.x()) / 2.0;
    target_box_center_pixel.y() = (lower_right.y() + upper_left.y()) / 2.0;

    // Generate alpha rotated 3D template here. Corners in Camera space:
    // Bottom: FL, FR, RR, RL => Top: FL, FR, RR, RL
    double deg_alpha = alpha_deg;
    double h_half = h / 2.0;
    double w_half = w / 2.0;
    double l_half = l / 2.0;

    std::vector<Eigen::Vector3d> corners;
    corners.resize(8);
    corners[0] = Eigen::Vector3d(l_half, h_half, w_half);
    corners[1] = Eigen::Vector3d(l_half, h_half, -w_half);
    corners[2] = Eigen::Vector3d(-l_half, h_half, -w_half);
    corners[3] = Eigen::Vector3d(-l_half, h_half, w_half);
    corners[4] = Eigen::Vector3d(l_half, -h_half, w_half);
    corners[5] = Eigen::Vector3d(l_half, -h_half, -w_half);
    corners[6] = Eigen::Vector3d(-l_half, -h_half, -w_half);
    corners[7] = Eigen::Vector3d(-l_half, -h_half, w_half);
    rotate_object(deg_alpha, corners);
    _corners = corners;

    // Try to get an initial Mass center pixel and vector
    Eigen::Matrix<double, 3, 1> middle_v(0.0, 0.0, 20.0);
    Eigen::Matrix<double, 2, 1> center_pixel =  _camera_model.project(middle_v);

    double max_pixel_x = std::numeric_limits<double>::min();
    double min_pixel_x = std::numeric_limits<double>::max();
    double max_pixel_y = std::numeric_limits<double>::min();
    double min_pixel_y = std::numeric_limits<double>::max();
    for (size_t i = 0; i < corners.size(); ++i) {
        Eigen::Vector2d point_2d = _camera_model.project(corners[i] + middle_v);
        min_pixel_x = std::min(min_pixel_x, point_2d.x());
        max_pixel_x = std::max(max_pixel_x, point_2d.x());
        min_pixel_y = std::min(min_pixel_y, point_2d.y());
        max_pixel_y = std::max(max_pixel_y, point_2d.y());
    }
    double relative_x = (center_pixel.x() - min_pixel_x) / (max_pixel_x - min_pixel_x);
    double relative_y = (center_pixel.y() - min_pixel_y) / (max_pixel_y - min_pixel_y);

    mass_center_pixel.x() = (lower_right.x() - upper_left.x()) * relative_x + upper_left.x();
    mass_center_pixel.y() = (lower_right.y() - upper_left.y()) * relative_y + upper_left.y();
    Eigen::Matrix<double, 3, 1> mass_center_v = _camera_model.unproject(mass_center_pixel);
    mass_center_v = to_unit_v(mass_center_v);

    // Binary search
    distance_w = distance_binary_search(pixel_width, true, mass_center_v);
    distance_h = distance_binary_search(pixel_height, false, mass_center_v);

    for (size_t i = 0; i < 2; ++i) {
        // Mass center search
        mass_center_search(target_box_center_pixel, distance_h, mass_center_v, mass_center_pixel);

        // Binary search
        distance_w = distance_binary_search(pixel_width, true, mass_center_v);
        distance_h = distance_binary_search(pixel_height, false, mass_center_v);
    }

    return true;
}

void GeometryCameraTransformer::set_visualization(bool flag) {
    _visualization = flag;
}

std::string GeometryCameraTransformer::name() const {
    return "GeometryCameraTransformer";
}

bool GeometryCameraTransformer::load_camera_intrinsics(const std::string &file_name) {

    YAML::Node node = YAML::LoadFile(file_name);

    Eigen::Matrix3d intrinsic_k;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            int index = i * 3 + j;
            intrinsic_k(i,j) = node["K"][index].as<double>();
        }
    }

    double height = node["height"].as<double>();
    double width = node["width"].as<double>();
    _camera_model.set(intrinsic_k, width, height);

    return true;
}

void GeometryCameraTransformer::rotate_object(double alpha_deg, std::vector<Eigen::Vector3d > &corners) const {

    Eigen::AngleAxisd yaw(alpha_deg / 180.0 * M_PI, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitch(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd roll(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation = yaw.toRotationMatrix() * pitch.toRotationMatrix() * roll.toRotationMatrix();

    Eigen::Matrix4d transform;
    transform.setIdentity();
    transform.block(0,0,3,3) = rotation;

    for (auto &corner: corners) {
        Eigen::Vector4d temp(corner.x(), corner.y(), corner.z(), 1.0);
        temp = transform * temp;
        corner = Eigen::Vector3d(temp.x(), temp.y(), temp.z());
    }
}

double GeometryCameraTransformer::distance_binary_search(const int &target_pixel_length, const bool &use_width,
                                                         const Eigen::Matrix<double, 3, 1> &mass_center_v) const {
    double close_d = 0.1;
    double far_d = 200.0;
    double curr_d = 0.0;
    int depth = 0;
    while (close_d <= far_d && depth < max_distance_binary_search_depth) {

        curr_d = (far_d + close_d) / 2.0;
        Eigen::Vector3d curr_p = mass_center_v * curr_d;

        double min_p = std::numeric_limits<double>::max();
        double max_p = 0.0;
        for (size_t i = 0; i < _corners.size(); ++i) {
            Eigen::Vector2d point_2d = _camera_model.project(_corners[i] + curr_p);

            double curr_pixel = 0.0;
            if (use_width) {
                curr_pixel = point_2d.x();
            }
            else {
                curr_pixel = point_2d.y();
            }

            min_p = std::min(min_p, curr_pixel);
            max_p = std::max(max_p, curr_pixel);
        }

        int curr_pixel_length = static_cast<int>(max_p - min_p);
        if (curr_pixel_length == target_pixel_length) {
            break;
        }
        else if (target_pixel_length < curr_pixel_length) {
            close_d = curr_d + 0.01;
        }
        else { // target_pixel_length > curr_pixel_length
            far_d = curr_d - 0.01;
        }

        // Early break for 0.01m accuracy
        double next_d = (far_d + close_d) / 2.0;
        if (std::abs(next_d - curr_d) < 0.01) {
            break;
        }

        ++depth;
    }

    return curr_d;
}

void GeometryCameraTransformer::mass_center_search(const Eigen::Matrix<double, 2, 1> &target_box_center_pixel,
                                                   const double &curr_d, Eigen::Matrix<double, 3, 1> &mass_center_v,
                                                   Eigen::Matrix<double, 2, 1> &mass_center_pixel) const {
    int depth = 0;
    while (depth < max_mass_center_search_depth) {

        Eigen::Matrix<double, 3, 1> new_center_v = mass_center_v * curr_d;

        double max_pixel_x = std::numeric_limits<double>::min();
        double min_pixel_x = std::numeric_limits<double>::max();
        double max_pixel_y = std::numeric_limits<double>::min();
        double min_pixel_y = std::numeric_limits<double>::max();
        for (size_t i = 0; i < _corners.size(); ++i) {
            Eigen::Vector2d point_2d = _camera_model.project(_corners[i] + new_center_v);
            min_pixel_x = std::min(min_pixel_x, point_2d.x());
            max_pixel_x = std::max(max_pixel_x, point_2d.x());
            min_pixel_y = std::min(min_pixel_y, point_2d.y());
            max_pixel_y = std::max(max_pixel_y, point_2d.y());
        }

        Eigen::Matrix<double, 2, 1> current_box_center_pixel;
        current_box_center_pixel.x() = (max_pixel_x + min_pixel_x) / 2.0;
        current_box_center_pixel.y() = (max_pixel_y + min_pixel_y) / 2.0;

        // Update mass center
        mass_center_pixel += target_box_center_pixel - current_box_center_pixel;
        mass_center_v = _camera_model.unproject(mass_center_pixel);
        mass_center_v = to_unit_v(mass_center_v);

        if (std::abs(mass_center_pixel.x() - target_box_center_pixel.x()) < 1.0
            && std::abs(mass_center_pixel.y() - target_box_center_pixel.y()) < 1.0) {
            break;
        }

        ++depth;
    }

    return;
}

Eigen::Matrix<double, 3, 1> GeometryCameraTransformer::to_unit_v(const Eigen::Matrix<double, 3, 1> &v) const {
    Eigen::Matrix<double, 3, 1> unit_v = v;
    double to_unit_scale = std::sqrt(unit_v.x() * unit_v.x()
                                     + unit_v.y() * unit_v.y()
                                     + unit_v.z() * unit_v.z());
    unit_v /= to_unit_scale;
    return unit_v;
};

// Register plugin.
REGISTER_CAMERA_TRANSFORMER(GeometryCameraTransformer);

} //namespace adu
} //namespace perception
} //namespace obstacle
