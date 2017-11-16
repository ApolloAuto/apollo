//
// Created by gaohan02 on 16-9-13.
//

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_UNITY_RECTIFY_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_UNITY_RECTIFY_H

#include "modules/perception/traffic_light/interface/base_rectifier.h"
#include "modules/perception/traffic_light/interface/green_interface.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class UnityRectify : public BaseRectifier {
 public:
  UnityRectify() = default;

  virtual bool Init() override;

  // @brief: rectify light region from image or part of it
  // @param [in] const Image&: input image
  // @param [in] const RectifyOptions&: rectify options
  // @param [in/out] Lights
  // @return  bool
  virtual bool Rectify(const Image &image, const RectifyOption &option,
                       std::vector<LightPtr> *lights) override;

  bool InitDetection(const ConfigManager *config_manager,
                     const ModelConfig *model_config,
                     std::shared_ptr<IRefine> *detection, std::shared_ptr<IGetBox> *crop);

  // @brief name
  virtual std::string name() const;

 private:
  std::shared_ptr<ISelectLight> select_;
  std::shared_ptr<IRefine> detect_;
  std::shared_ptr<IGetBox> crop_;
};

REGISTER_RECTIFIER(UnityRectify);
}
}
}

#endif //PERCEPTION_DENSEBOXRECTIFY_H
