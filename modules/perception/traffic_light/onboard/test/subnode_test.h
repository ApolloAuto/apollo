// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(erlangz@baidu.com)
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TEST_SUBNODE_TEST_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TEST_SUBNODE_TEST_H

#include <gtest/gtest.h>
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class SubnodeTest : public testing::Test {
 public:
  SubnodeTest() = default;
  virtual ~SubnodeTest() = default;

 protected:
  bool init_event_manager(const std::vector<std::string> &event_name,
                          std::vector<EventMeta> *event_meta,
                          EventManager *event_manager);
  bool init_shared_data_manager(const std::vector<std::string> &share_data_names,
                                SharedDataManager *data_manager);
 protected:
  EventManager event_manager;
  SharedDataManager shared_data_manager;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TEST_SUBNODE_TEST_H
// @date 2016/09/20 14:30:54

