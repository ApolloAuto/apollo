//
// Created by gaohan02 on 17-4-17.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_ACTION_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_ACTION_H

#include "modules/perception/onboard/common_shared_data.h"
#include "modules/perception/onboard/subnode.h"

namespace apollo {
namespace perception {
namespace traffic_light {
//@brief the Action of Msg synced.
class BaseTLAction {
 public:
  //@brief pub action name
  virtual std::string name() const =0;

  //@brief the action when data has been synced.
  virtual bool action(const ImageSharedPtr &long_focus_image,
                      const ImageSharedPtr &short_focus_image,
                      const double ts)=0;
};
}
}
}
#endif //ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_ACTION_H
