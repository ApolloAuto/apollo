#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_LOCATION_EXPORTER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_LOCATION_EXPORTER_H

#include "modules/localization/msf/local_tool/data_extraction/base_exporter.h"

namespace apollo {
namespace localization {
namespace msf {
class LocationExporter : public BaseExporter {
public:
    typedef std::shared_ptr<LocationExporter> Ptr;
	typedef std::shared_ptr<LocationExporter const> ConstPtr;

    LocationExporter(const std::string &loc_file_folder);
    ~LocationExporter();

    void GnssLocCallback(const rosbag::MessageInstance &msg);
    void LidarLocCallback(const rosbag::MessageInstance &msg);
    void FusionLocCallback(const rosbag::MessageInstance &msg);

private:
    std::string _gnss_loc_file;
    std::string _lidar_loc_file;
    std::string _fusion_loc_file;

    FILE* _gnss_loc_file_handle;
    FILE* _lidar_loc_file_handle;
    FILE* _fusion_loc_file_handle;
};

} // msf
} // localization
} // apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_LOCATION_EXPORTER_H
