
#pragma once
#include "modules/drivers/lidar/vanjeelidar/common/common_header.h"

namespace apollo {
namespace drivers {
namespace vanjee {

template <typename PointT>
#ifdef _MSC_VER
struct __declspec(align(16)) PointCloudMsg
#elif __GNUC__
struct __attribute__((aligned(16))) PointCloudMsg
#endif
{
  typedef std::vector<PointT> PointCloud;
  typedef std::shared_ptr<PointCloud> PointCloudPtr;
  typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;
  double timestamp = 0.0;
  std::string frame_id = "";      ///< Point cloud frame id
  uint32_t seq = 0;               ///< Sequence number of message
  uint32_t height = 0;            ///< Height of point cloud
  uint32_t width = 0;             ///< Width of point cloud
  bool is_dense = false;          ///< If is_dense=true, the point cloud does not contain NAN points
  PointCloudPtr point_cloud_ptr;  ///< Point cloud pointer
  PointCloudMsg() = default;
  explicit PointCloudMsg(const PointCloudPtr& ptr) : point_cloud_ptr(ptr)
  {
  }
  typedef std::shared_ptr<PointCloudMsg> Ptr;
  typedef std::shared_ptr<const PointCloudMsg> ConstPtr;
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo