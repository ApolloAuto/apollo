//
// Created by gaohan02 on 16-9-20.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_RECOGNIZE_UNITY_RECOGNIZE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_RECOGNIZE_UNITY_RECOGNIZE_H

#include <traffic_light/interface/base_recognizer.h>
#include <traffic_light/interface/green_interface.h>
#include <lib/config_manager/config_manager.h>

namespace adu {
namespace perception {
namespace traffic_light {
class UnityRecognize : public BaseRecognizer {
 public:
  UnityRecognize() {
  }

  virtual bool init();

  // @brief: recognize light status
  // @param [in] const Recognize&: recognize options
  // @param [in] const Image&: input image
  // @param [in/out] std::vector<Light>*: recognized light status
  // @return  bool
  virtual bool recognize_status(const Image &image, const RecognizeOption &option,
                                std::vector<LightPtr> *lights) override;

  virtual std::string name() const;

 private:
  std::shared_ptr<IRefine> _classify_day;
  std::shared_ptr<IRefine> _classify_night;

  bool init_model(const adu::perception::config_manager::ConfigManager *config_manager,
                  const adu::perception::config_manager::ModelConfig *model_config,
                  std::shared_ptr<IRefine> *classify);
};
}
}
}
#endif //PERCEPTION_COLOR_RECOGNIZE_H
