#ifndef  MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ROSBAG_READER_H
#define  MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ROSBAG_READER_H
#include <string>
#include "modules/localization/msf/local_tool/data_extraction/base_exporter.h"
#include <unordered_map>  

namespace apollo {
namespace localization {
namespace msf {
class RosbagReader {
public:
    RosbagReader();
    ~RosbagReader();

    void Subscribe(const std::string& topic, BaseExporter::OnRosmsgCallback call_back,
                   BaseExporter::Ptr exporter);
    void Read(const std::string &file_name);

private:
    std::vector<std::string> _topics;
    std::unordered_map<std::string, 
        std::pair<BaseExporter::Ptr, BaseExporter::OnRosmsgCallback>> _call_back_map;
};

} // namespace
} // localization
} // apollo 
#endif // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ROSBAG_READER_H
