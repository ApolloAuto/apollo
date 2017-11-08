//
// Created by gaohan02 on 16-9-13.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_DENSEBOX_RECTIFY_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_DENSEBOX_RECTIFY_H

#include <traffic_light/interface/base_rectifier.h>
#include <traffic_light/interface/green_interface.h>
#include <lib/config_manager/config_manager.h>

namespace adu {
namespace perception {
namespace traffic_light {

class DenseBoxRectify : public BaseRectifier {
 public:
  DenseBoxRectify() {
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

 private:
  std::shared_ptr<ISelectLight> _select;
  std::shared_ptr<IRefine> _detect_day;
  std::shared_ptr<IRefine> _detect_night;
  std::shared_ptr<IRectify> _rectify;
  std::shared_ptr<IHDMapOperator> _verifymap;
  std::shared_ptr<IGetBox> _crop_day;
  std::shared_ptr<IGetBox> _crop_night;
  std::vector<cv::KalmanFilter> _kf;
  std::vector<bool> _init;
  bool _enable_track = false;
  int _night_begin;
  int _night_end;
};
}
}
}

#endif //PERCEPTION_DENSEBOXRECTIFY_H
