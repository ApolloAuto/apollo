#ifndef APOLLO_LOCALIZATION_EULER_TF_H
#define APOLLO_LOCALIZATION_EULER_TF_H

#include "sins_struct.h"
namespace apollo {
namespace localization {
namespace msf {
void quaternion_to_euler(const double quaternion[4], Attitude *att);
void euler_to_quaternion(const double x_euler, const double y_euler, 
                         const double z_euler, double qbn[4]);
void euler_to_dcm(const Attitude *att, double dcm[3][3]);
void quaternion_to_dcm(const double *quaternion, double dcm[3][3]);


}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif