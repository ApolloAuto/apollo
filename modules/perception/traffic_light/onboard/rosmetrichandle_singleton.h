// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/08/30
// @file: rosmetrichandle_singleton.h
// @brief: 
// 
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_ROSMETRICHANDLE_SINGLETON_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_ROSMETRICHANDLE_SINGLETON_H

#include <xlog.h>
#include "lib/base/singleton.h"
#include "roslibmetric/metric_handle.h"

namespace adu {
namespace perception {
namespace traffic_light {

class RosMetricHandleSingleton {
 public:
  void throw_exception(uint64_t mode_num, uint64_t fault_num, std::string msg) {
    _m.throw_exception(mode_num, fault_num, msg);
  }

 private:
  RosMetricHandleSingleton() : _m(ros::get_metric_handle()) {}
  ~RosMetricHandleSingleton() {}
  friend class ::adu::perception::base::Singleton<RosMetricHandleSingleton>;

  ros::MetricHandle &_m;
};

}
}
}

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_ROSMETRICHANDLE_SINGLETON_H
