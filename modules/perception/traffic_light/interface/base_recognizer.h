// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: Hengyu Li (lihengyu@baidu.com)
// @file: base_recognizer.h
// @brief: interface of light status recognizer

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECOGNIZER_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECOGNIZER_H

#include <string>

#include <opencv2/opencv.hpp>

#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct RecognizeOption {

};

//@brief Recognizer classify the light color. 
class BaseRecognizer {
 public:
  BaseRecognizer() = default;

  virtual ~BaseRecognizer() = default;

  virtual bool Init() = 0;

  // @brief: recognize light status
  // @param [in] const Recognize&: recognize options
  // @param [in] const Image&: input image
  // @param [in/out] std::vector<Light>*: recognized light status
  // @return  bool
  virtual bool RecognizeStatus(const Image &image, const RecognizeOption &option,
                               std::vector<LightPtr> *lights) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseRecognizer);
};

REGISTER_REGISTERER(BaseRecognizer);
#define REGISTER_RECOGNIZER(name) REGISTER_CLASS(BaseRecognizer, name)

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECOGNIZER_H
