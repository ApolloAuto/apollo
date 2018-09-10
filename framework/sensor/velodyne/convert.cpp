
#include "cybertron/common/log.h"
#include "sensor/velodyne/convert.h"

namespace apollo {
namespace sensor {
namespace velodyne {

/** @brief Constructor. */
Convert::Convert(proto::VelodyneConfig& velodyne_config) {

  _config = velodyne_config;
  _config.set_view_direction(0.0);
  _config.set_view_width(2.0 * M_PI);

  _parser = VelodyneParserFactory::create_parser(_config);
  if (_parser == nullptr) {
    AERROR << "can not create velodyen parser";
  } else {
    _parser->setup();
  }
}

Convert::~Convert() {
  if (_parser != nullptr) {
    delete _parser;
  }
}

/** @brief Callback for raw scan messages. */
void Convert::convert_velodyne_to_pointcloud(
    const std::shared_ptr<proto::VelodyneScan const>& scan_msg,
    std::shared_ptr<proto::PointCloud>& point_cloud) {

  _parser->generate_pointcloud(scan_msg, point_cloud);

  if (point_cloud == nullptr || point_cloud->point_size() == 0) {
    AERROR << "point cloud has no point";
    return;
  }

  if (_config.organized()) {
    _parser->order(point_cloud);
  }

  // pcl::PCLPointCloud2 pcl_cloud;
  // pcl_conversions::toPCL(*msg, pcl_cloud);
  // pcl::PCDWriter writer;
  // writer.writeBinaryCompressed(filename, pcl_cloud);
}

}  // namespace 
}
}
