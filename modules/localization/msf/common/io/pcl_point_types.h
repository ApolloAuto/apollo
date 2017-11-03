#ifndef IDL_CAR_PCL_POINT_TYPES_H
#define IDL_CAR_PCL_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace apollo {
namespace localization {
namespace msf {
namespace velodyne {

struct PointXYZIRT {
  float x;
  float y;
  float z;
  unsigned char intensity;
  unsigned char ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

struct PointXYZIT {
  float x;
  float y;
  float z;
  unsigned char intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

struct PointXYZIRTd {
  double x;
  double y;
  double z;
  unsigned char intensity;
  unsigned char ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

struct PointXYZITd {
  double x;
  double y;
  double z;
  unsigned char intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

} // namespace velodyne
} // namespace msf
} // namespace localization
} // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT ( apollo::localization::msf::velodyne::PointXYZIT,
                                    ( float, x, x )
                                    ( float, y, y )
                                    ( float, z, z )
                                    ( uint8_t, intensity, intensity )
                                    ( double, timestamp, timestamp )
                                  )

POINT_CLOUD_REGISTER_POINT_STRUCT ( apollo::localization::msf::velodyne::PointXYZIRT,
                                    ( float, x, x )
                                    ( float, y, y )
                                    ( float, z, z )
                                    ( uint8_t, intensity, intensity )
                                    ( uint8_t, ring, ring )
                                    ( double, timestamp, timestamp )
                                  )

POINT_CLOUD_REGISTER_POINT_STRUCT ( apollo::localization::msf::velodyne::PointXYZITd,
                                    ( double, x, x )
                                    ( double, y, y )
                                    ( double, z, z )
                                    ( uint8_t, intensity, intensity )
                                    ( double, timestamp, timestamp )
                                  )

POINT_CLOUD_REGISTER_POINT_STRUCT ( apollo::localization::msf::velodyne::PointXYZIRTd,
                                    ( double, x, x )
                                    ( double, y, y )
                                    ( double, z, z )
                                    ( uint8_t, intensity, intensity )
                                    ( uint8_t, ring, ring )
                                    ( double, timestamp, timestamp )
                                  )

#endif // IDL_CAR_PCL_POINT_TYPES_H

