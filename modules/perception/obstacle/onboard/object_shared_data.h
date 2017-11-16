#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBJECT_SHARED_DATA_H
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBJECT_SHARED_DATA_H

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/onboard/common_shared_data.h"

namespace apollo {
namespace perception {

#define OBJECT_SHARED_DATA(data_name)  \
class data_name : public CommonSharedData<SensorObjects> {  \
public:  \
    data_name() : CommonSharedData<SensorObjects>() {}  \
    virtual ~data_name() {}  \
    virtual std::string name() const override {  \
        return #data_name;  \
    }  \
private:  \
    DISALLOW_COPY_AND_ASSIGN(data_name);  \
}

OBJECT_SHARED_DATA(LidarObjectData);
OBJECT_SHARED_DATA(RadarObjectData);

REGISTER_SHAREDDATA(LidarObjectData);
REGISTER_SHAREDDATA(RadarObjectData);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBJECT_SHARED_DATA_H
