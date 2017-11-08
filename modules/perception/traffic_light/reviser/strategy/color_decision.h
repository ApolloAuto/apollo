//
// Created by gaohan02 on 16-12-2.
//

#ifndef PERCEPTION_COLOR_DECISION_H
#define PERCEPTION_COLOR_DECISION_H

#include <traffic_light/interface/green_interface.h>
#include "modules/perception/traffic_light/interface/base_reviser.h"

namespace adu {
namespace perception {
namespace traffic_light {

class ColorReviser : public BaseReviser {
 public:
  ColorReviser() {
  }

  //@brief init the reviser.
  virtual bool init();
  //@brief reviser revise  the perception result
  //       ASSERT(rectifed_result.size == perception_result.size)
  //@param [in] option
  //@param [in/out] rectifed_result
  //@return true/false
  virtual bool revise(const ReviseOption &option, std::vector<LightPtr> *lights) override;

  //@brief Revise's name
  virtual std::string name() const;

 private:
  float _blink_time;
  int _enable;
  std::map<std::string, TLColor> _color_map;
  std::map<std::string, double> _time_map;
  static std::map<TLColor, std::string> _s_color_strs;
};

}
}
}

#endif //PERCEPTION_COLOR_DECISION_H
