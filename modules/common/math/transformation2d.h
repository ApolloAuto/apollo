/*
 * transformation2d.h
 *
 *  Created on: Jul 28, 2017
 *      Author: zhangyajia
 */

#ifndef MODULES_COMMON_MATH_TRANSFORMATION2D_H_
#define MODULES_COMMON_MATH_TRANSFORMATION2D_H_

#include "Eigen/Dense"

namespace apollo {
namespace common {
namespace math {

class Transformation2d {
public:
//    static void transform(Eigen::Vector2d& p1, const double theta1, Eigen::Vector2d& p2, const double theta2, double* x, double* y, double* theta);
//
//    static void transform(const double x, const double y, const double tx, const double ty, const double theta, double* x_out, double* y_out);
//
//    static Eigen::Vector2d transform(const Eigen::Vector2d& p, const Eigen::Vector2d& t, const double theta);

    Transformation2d(const Eigen::Vector2d& t, const double theta);

    void rotate(const double theta);

    void translate(const Eigen::Vector2d& t);

    void transform(const Transformation2d& transformation);

    void transform(const double theta, const Eigen::Vector2d& t);

    Eigen::Vector2d translation() const;

    double theta() const;

    Eigen::Matrix<double, 3, 3> to_transformation_matrix() const;
private:
    Eigen::Vector2d translation_;

    double theta_;
};

class Rotation2d {
public:
    static Eigen::Vector2d rotate(const Eigen::Vector2d& p, const double theta);

    static Eigen::Matrix<double, 2, 2> to_rotation_matrix(const double theta);
};
} //namespace math
} //namespace common
} //namespace apollo

#endif /* MODULES_COMMON_MATH_TRANSFORMATION2D_H_ */
