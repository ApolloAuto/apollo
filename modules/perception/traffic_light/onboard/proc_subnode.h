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
#include "modules/perception/onboard/common_shared_data.h"
#include "modules/perception/traffic_light/interface/green_interface.h"

#include "modules/perception/onboard/subnode.h"
#include "modules/perception/traffic_light/interface/base_rectifier.h"
#include "modules/perception/traffic_light/interface/base_recognizer.h"
#include "modules/perception/traffic_light/interface/base_reviser.h"
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

DECLARE_string(traffic_light_rectifier);
DECLARE_string(traffic_light_recognizer);
DECLARE_string(traffic_light_reviser);

class ImageLights;
class TLPreprocessingData;
class TLProcData;

class TLProcSubnode : public CommonSubnode {
 public:
  TLProcSubnode() = default;
  virtual ~TLProcSubnode();

 protected:
  virtual bool InitInternal() override;
  virtual bool HandleEvent(const Event &sub_event,
                           Event *pub_event) override;

 private:
  bool InitSharedData();
  bool InitRectifier();
  bool InitRecognizer();
  bool InitReviser();

  //get mean distance from car to stopline.
  double GetMeanDistance(const double ts,
                         const Eigen::Matrix4d &car_location,
                         const LightPtrs &lights) const;

  bool VerifyImageLights(const ImageLights &image_lights, CameraId *selection) const;

  //@brief compute image border size based on projection box and detection box
  bool ComputeImageBorder(const ImageLights &image_lights, int *image_border);

  //@brief compute offset between two rectangles based on their relative positions
  void ComputeRectsOffset(const cv::Rect &rect1, const cv::Rect &rect2, int *offset);

 private:
  int image_border_;

  TLPreprocessingData *preprocessing_data_ = nullptr;  // up-stream data
  TLProcData *proc_data_ = nullptr;       // down-stream data

  std::unique_ptr<BaseRectifier> rectifier_ = nullptr;
  std::unique_ptr<BaseRecognizer> recognizer_ = nullptr;
  std::unique_ptr<BaseReviser> reviser_ = nullptr;
  std::shared_ptr<IGetBox> crop_;
  Mutex mutex_;
  DISALLOW_COPY_AND_ASSIGN(TLProcSubnode);
};

REGISTER_SUBNODE(TLProcSubnode);
} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROC_SUBNODE_H
