
#ifndef SENSOR_VELODYNE_COMPENSATOR_H
#define SENSOR_VELODYNE_COMPENSATOR_H 

#include <Eigen/Dense>
#include "cybertron/tf2_cybertron/transform_listener.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"
#include "cybertron/time/time.h"
#include "sensor/proto/sensor_pointcloud.pb.h"
#include "sensor/proto/velodyne_config.pb.h"

#include "sensor/velodyne/const_variables.h"

namespace apollo {
namespace sensor {
namespace velodyne {

class Compensator {

 public:
  explicit Compensator(proto::VelodyneConfig& velodyne_config);
  virtual ~Compensator() {}

  /**
  * @brief get pointcloud2 msg, compensate it,publish pointcloud2 after
  * compensator
  *  TODO: why callback data type is sensor_msgs::PointCloud2, not
  * velodyne_parser::VPointCloud
  */
  bool motion_compensation(
      const std::shared_ptr<const proto::PointCloud>& msg,
      std::shared_ptr<proto::PointCloud>& msg_compensated);

 private:
  /**
  * @brief get pose affine from tf2 by gps timestamp
  *   novatel-preprocess broadcast the tf2 transfrom.
  */
  bool query_pose_affine_from_tf2(const uint64_t& timestamp,
                                  void* pose,
                                  const std::string& child_frame_id);

  bool query_pose(const uint64_t& timestamp,
    void* pose,
    const std::string& child_frame_id,
    const std::string& parent_frame_id);

  /**
  * @brief motion compensation for point cloud
  */
  void motion_compensation(
      const std::shared_ptr<const proto::PointCloud>& msg,
      std::shared_ptr<proto::PointCloud>& msg_compensated,
      const uint64_t timestamp_min, const uint64_t timestamp_max,
      const Eigen::Affine3d& pose_min_time,
      const Eigen::Affine3d& pose_max_time);
  /**
  * @brief get min timestamp and max timestamp from points in pointcloud2
  */
  inline void get_timestamp_interval(
      const std::shared_ptr<const proto::PointCloud>& msg,
      uint64_t& timestamp_min, uint64_t& timestamp_max);

  bool is_valid(Eigen::Vector3d& point);

  std::shared_ptr<apollo::cybertron::tf2_cybertron::Buffer> _tf2_buffer_ptr;
//  apollo::cybertron::tf2_cybertron::TransformListener _tf2_transform_listener;
  proto::VelodyneConfig _config;
};

}  // namespace velodyne
}
}

#endif  // ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_PARSER_COMPENSATOR_H
