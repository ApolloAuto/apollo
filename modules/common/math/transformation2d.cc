/*
 * transformation2d.cc
 *
 *  Created on: Jul 28, 2017
 *      Author: zhangyajia
 */

#include "transformation2d.h"

namespace apollo {
namespace common {
namespace math {

Transformation2d::Transformation2d(const Eigen::Vector2d& t,
        const double theta) {
}

void Transformation2d::rotate(const double theta) {
}

void Transformation2d::translate(const Eigen::Vector2d& t) {
}

void Transformation2d::transform(const Transformation2d& transformation) {
}

void Transformation2d::transform(const double theta,
        const Eigen::Vector2d& t) {
}

Eigen::Vector2d Transformation2d::translation() const {
}

double Transformation2d::theta() const {
}

Eigen::Matrix<double, 3, 3> Transformation2d::to_transformation_matrix() const {
}

//void Transformation2d::transform(const double x1, const double y1, const double theta1,
//        const double x2, const double y2, const double theta2, double* x, double* y,
//        double* theta) {
//
//    transform(x2, y2, x1, y1, theta1, x, y);
//    *theta = math_util::MathUtil::normalize_angle(theta1 + theta2);
//}
//
//void Transformation2d::transform(const double x, const double y, const double tx,
//        const double ty, const double theta, double* x_out, double* y_out) {
//    Rotation2d::rotate(x, y, theta, x_out, y_out);
//    *x_out += tx;
//    *y_out += ty;
//    return;
//}
//
//Eigen::Vector2d Transformation2d::transform(const Eigen::Vector2d& p, const double tx,
//        const double ty, const double theta) {
//    Eigen::Vector2d p_out = Rotation2d::rotate(p, theta);
//    p_out[0] += tx;
//    p_out[1] += ty;
//    return p_out;
//}
//
//Eigen::Vector2d Transformation2d::transform(const Eigen::Vector2d& p, const Eigen::Vector2d& t,
//        const double theta) {
//    Eigen::Vector2d p_out = Rotation2d::rotate(p, theta) + t;
//    return p_out;
//}

Eigen::Vector2d Rotation2d::rotate(const Eigen::Vector2d& p, const double theta) {
    double x = p.x();
    double y = p.y();
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    double x_out = cos_theta * x - sin_theta * y;
    double y_out = sin_theta * x + cos_theta * y;

    return {x_out, y_out};
}

Eigen::Matrix<double, 2, 2> Rotation2d::to_rotation_matrix(const double theta) {
}

}
}
}
