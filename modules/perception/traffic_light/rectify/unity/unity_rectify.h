//
// Created by gaohan02 on 16-9-13.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_UNITY_RECTIFY_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_UNITY_RECTIFY_H

#include <traffic_light/interface/base_rectifier.h>
#include <traffic_light/interface/green_interface.h>
#include <lib/config_manager/config_manager.h>
#include "module/perception/traffic_light/rectify/unity/detection.h"

namespace adu {
namespace perception {
namespace traffic_light {

class UnityRectify : public BaseRectifier {
 public:
  UnityRectify() {
  }

  virtual bool init() override;

  // @brief: rectify light region from image or part of it
  // @param [in] const Image&: input image
  // @param [in] const RectifyOptions&: rectify options
  // @param [in/out] Lights
  // @return  bool
  virtual bool rectify(const Image &image, const RectifyOption &option,
                       std::vector<LightPtr> *lights) override;

  bool init_detection(const config_manager::ConfigManager *config_manager,
                      const config_manager::ModelConfig *model_config,
                      std::shared_ptr<IRefine> *detection, std::shared_ptr<IGetBox> *crop);

  // @brief name
  virtual std::string name() const;

  bool set_output_box_type(DetectOutputBoxType type);

 private:
  std::shared_ptr<ISelectLight> _select;
  std::shared_ptr<IRefine> _detect;
  std::shared_ptr<IHDMapOperator> _verifymap;
  std::shared_ptr<IGetBox> _crop;
};

}
}
}

#endif //PERCEPTION_DENSEBOXRECTIFY_H
