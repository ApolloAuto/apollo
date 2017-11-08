// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/08/08
// @file: proc_subnode.h
// @brief: 
// 
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROC_SUBNODE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROC_SUBNODE_H

#include <map>
#include <cmath>
#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <onboard/common_shared_data.h>
#include <traffic_light/interface/green_interface.h>

#include "onboard/subnode.h"
#include "onboard/transform_input_wrapper.h"
#include "module/perception/traffic_light/interface/base_rectifier.h"
#include "module/perception/traffic_light/interface/base_recognizer.h"
#include "module/perception/traffic_light/interface/base_reviser.h"
#include "module/perception/traffic_light/projection/multi_camera_projection.h"

namespace adu {
namespace perception {
namespace traffic_light {

DECLARE_string(traffic_light_rectifier);
DECLARE_string(traffic_light_recognizer);
DECLARE_string(traffic_light_reviser);

class ImageLights;
class TLPreprocessingData;
class TLProcData;

class TLProcSubnode : public onboard::CommonSubnode {
 public:
  TLProcSubnode() = default;
  virtual ~TLProcSubnode();

 protected:
  virtual bool init_internal() override;
  virtual bool handle_event(const onboard::Event &sub_event,
                            onboard::Event *pub_event) override;

 private:
  bool init_shared_data();
  bool init_rectifier();
  bool init_recognizer();
  bool init_reviser();

  //get mean distance from car to stopline.
  double get_mean_distance(const double ts,
                           const Eigen::Matrix4d &car_location,
                           const LightPtrs &lights) const;

  bool verify_image_lights(const ImageLights &image_lights, CameraId *selection) const;

  //@brief compute image border size based on projection box and detection box
  bool compute_image_border(const ImageLights &image_lights, int *image_border);

  //@brief compute offset between two rectangles based on their relative positions
  void compute_rects_offset(const cv::Rect &rect1, const cv::Rect &rect2, int *offset);

 private:
  int _image_border;

  TLPreprocessingData *_preprocessing_data = nullptr;  // up-stream data
  TLProcData *_proc_data = nullptr;       // down-stream data

  std::unique_ptr<BaseRectifier> _rectifier = nullptr;
  std::unique_ptr<BaseRecognizer> _recognizer = nullptr;
  std::unique_ptr<BaseReviser> _reviser = nullptr;
  std::shared_ptr<IGetBox> _crop;
  base::Mutex _mutex;
  DISALLOW_COPY_AND_ASSIGN(TLProcSubnode);
};

} // namespace traffic_light
} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROC_SUBNODE_H
