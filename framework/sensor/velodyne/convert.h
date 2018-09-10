
#ifndef SENSOR_VELODYNE_PARSER_H_
#define SENSOR_VELODYNE_PARSER_H_ 

#include "sensor/velodyne/velodyne_parser.h"
#include "sensor/proto/sensor_pointcloud.pb.h"
#include "sensor/proto/sensor_velodyne.pb.h"

namespace apollo {
namespace sensor {
namespace velodyne {

// convert velodyne data to pointcloud and republish
class Convert {

 public:
  explicit Convert(proto::VelodyneConfig& velodyne_config);
  ~Convert();

  void convert_velodyne_to_pointcloud(
      const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
      std::shared_ptr<proto::PointCloud>& point_cloud);

 private:
  void init_config();
  VelodyneParser* _parser;
  proto::VelodyneConfig _config;
};

}  // namespace velodyne
}
}

#endif  // SENSOR_VELODYNE_PARSER_H_ 
