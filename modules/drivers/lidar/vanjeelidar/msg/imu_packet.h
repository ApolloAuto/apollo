
#pragma once
#include "modules/drivers/lidar/vanjeelidar/common/common_header.h"

namespace apollo {
namespace drivers {
namespace vanjee {
#ifdef _MSC_VER
struct __declspec(align(16)) ImuPacket
#elif __GNUC__
struct __attribute__((aligned(16))) ImuPacket
#endif
{
  float64 timestamp = 0.0; 
  uint32 seq = 0;
  std::array<float64,4> orientation;
  std::array<float64,9> orientation_covariance;

  std::array<float64,3> angular_voc;
  std::array<float64,9> angular_voc_covariance;

  std::array<float64,3> linear_acce;
  std::array<float64,9> linear_acce_covariance;
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo