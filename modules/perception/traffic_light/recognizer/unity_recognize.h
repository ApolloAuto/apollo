//
// Created by gaohan02 on 16-9-20.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_RECOGNIZE_UNITY_RECOGNIZE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_RECOGNIZE_UNITY_RECOGNIZE_H

#include "modules/perception/traffic_light/interface/base_recognizer.h"
#include "modules/perception/traffic_light/interface/green_interface.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace traffic_light {
class UnityRecognize : public BaseRecognizer {
 public:
  UnityRecognize() = default;

  bool Init() override;

  // @brief: recognize light status
  // @param [in] const Recognize&: recognize options
  // @param [in] const Image&: input image
  // @param [in/out] std::vector<Light>*: recognized light status
  // @return  bool
  bool RecognizeStatus(const Image &image, const RecognizeOption &option,
                       std::vector<LightPtr> *lights) override;

  virtual std::string name() const;

 private:
  std::shared_ptr<IRefine> classify_day_;
  std::shared_ptr<IRefine> classify_night_;

  bool InitModel(const apollo::perception::ConfigManager *config_manager,
                 const apollo::perception::ModelConfig *model_config,
                 std::shared_ptr<IRefine> *classify);
};
}
}
}
#endif //PERCEPTION_COLOR_RECOGNIZE_H
