// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author Ruidong Tang (tangruidong@baidu.com)
// @file: output_subnode.h
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_OUTPUT_SUBNODE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_OUTPUT_SUBNODE_H

#include <map>
#include <string>
#include <array>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include "modules/perception/onboard/subnode.h"
#include "modules/perception/traffic_light/base/image_lights.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class TLProcData;

class TLOutputSubnode : public Subnode {
 public:
  TLOutputSubnode() = default;
  virtual ~TLOutputSubnode() {
    _proc_data = nullptr;
  }

  virtual StatusCode proc_events() override;

 protected:
  virtual bool init_internal() override;
 private:
  bool init_shared_data();
  bool init_output_stream();

  bool transform_message(const Event &event,
                         const std::shared_ptr<ImageLights> &lights,
                         boost::shared_ptr<std_msgs::String> *msg) const;
  bool proc_upstream_data(const Event &event);

 private:
  std::unique_ptr<StreamOutput> _output_stream;
  TLProcData *_proc_data = nullptr;

  // save BGR colors
  //static std::map<std::string, std::array<int, 3> > _s_color_table;

  DISALLOW_COPY_AND_ASSIGN(TLOutputSubnode);
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_OUTPUT_SUBNODE_H
