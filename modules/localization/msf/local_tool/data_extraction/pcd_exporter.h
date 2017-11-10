#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H

#include "modules/localization/msf/local_tool/data_extraction/base_exporter.h"
#include <sensor_msgs/PointCloud2.h>

namespace apollo {
namespace localization {
namespace msf {
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

    std::string _pcd_folder;
    FILE* _stamp_file_handle;
};

} // msf
} // localization
} // apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H
