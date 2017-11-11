

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H

#include <sensor_msgs/PointCloud2.h>
#include "modules/localization/msf/local_tool/data_extraction/base_exporter.h"

namespace apollo {
namespace localization {
namespace msf {

/**
 * @class PCDExporter
 * @brief Export pcd from rosbag.
 */
class PCDExporter : public BaseExporter {
 public:
  typedef std::shared_ptr<PCDExporter> Ptr;
  typedef std::shared_ptr<PCDExporter const> ConstPtr;

  PCDExporter(const std::string &pcd_folder);
  ~PCDExporter();

  void CompensatedPcdCallback(const rosbag::MessageInstance &msg);

 private:
  void WritePcdFile(const std::string &filename,
                    const sensor_msgs::PointCloud2::ConstPtr &msg);

  std::string pcd_folder_;
  FILE *stamp_file_handle_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H
