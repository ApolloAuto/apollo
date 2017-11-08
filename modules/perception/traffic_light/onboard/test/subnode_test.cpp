// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(erlangz@baidu.com)
// @date 2016/09/20 14:42:06
#include "module/perception/traffic_light/onboard/test/subnode_test.h"
#include <string>
#include <vector>
namespace adu {
namespace perception {
namespace traffic_light {

bool SubnodeTest::init_event_manager(const std::vector<std::string> &event_names,
                                     std::vector<onboard::EventMeta> *meta_events,
                                     onboard::EventManager *event_manager) {

  // to init edge_config
  onboard::DAGConfig::EdgeConfig edge_config;
  for (size_t i = 0; i < event_names.size(); i++) {
    onboard::EventMeta event_meta;
    event_meta.event_id = i + 1;
    event_meta.name = event_names[i];
    event_meta.from_node = i + 1;
    event_meta.to_node = i + 2;
    meta_events->push_back(event_meta);

    onboard::DAGConfig::Edge *edge = edge_config.add_edges();
    edge->set_id(i + 1);
    edge->set_from_node(i + 1);
    edge->set_to_node(i + 2);
    onboard::DAGConfig::Event *event = edge->add_events();
    event->set_id(i + 1);
    event->set_name(event_names[i]);
  }
  if (!event_manager->init(edge_config)) {
    AERROR << "SubnodeTest init event_manager failed.";
    return false;
  }
  return true;
}

bool SubnodeTest::init_shared_data_manager(const std::vector<std::string> &share_data_names,
                                           onboard::SharedDataManager *data_manager) {

  onboard::DAGConfig::SharedDataConfig conf;
  for (size_t i = 0; i < share_data_names.size(); i++) {
    onboard::DAGConfig::SharedData *data = conf.add_datas();
    data->set_id(i + 1);
    data->set_name(share_data_names[i]);
  }
  if (!data_manager->init(conf)) {
    AERROR << "SubnodeTest init data_manager failed.";
    return false;
  }
  return true;
}

} // namespace traffic_light
} // namespace perception
} // namespace adu
