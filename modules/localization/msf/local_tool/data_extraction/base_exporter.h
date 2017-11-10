#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_BASE_EXPORTER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_BASE_EXPORTER_H

#include <rosbag/bag.h>

namespace apollo {
namespace localization {
namespace msf {
class BaseExporter {

public:
    typedef void (BaseExporter::*OnRosmsgCallback)(const rosbag::MessageInstance &msg);

    typedef std::shared_ptr<BaseExporter> Ptr;
	typedef std::shared_ptr<BaseExporter const> ConstPtr;
protected:
    BaseExporter()
    {
    }

    ~BaseExporter(){}
};

} // msf
} // localization
} // apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_BASE_EXPORTER_H
