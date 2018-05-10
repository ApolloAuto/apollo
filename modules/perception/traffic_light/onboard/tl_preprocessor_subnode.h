/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TL_PREPROCESSOR_SUBNODE_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TL_PREPROCESSOR_SUBNODE_H_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "tf/transform_listener.h"
#include "tf2_ros/transform_listener.h"

#include "modules/common/time/timer.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"
#include "modules/perception/traffic_light/onboard/hdmap_input.h"
#include "modules/perception/traffic_light/preprocessor/tl_preprocessor.h"
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

/** @class TLPreprocessorSubnode
 *  @brief pre-processor subnode
 */
class TLPreprocessorSubnode : public Subnode {
 public:
  TLPreprocessorSubnode() = default;
  virtual ~TLPreprocessorSubnode() = default;

  /**
   * @brief as a subnode with type SUBNODE_IN
   *         we will use ros callback, so ignore subnode callback
   */
  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 protected:
  /**
   * @brief init pre-processor
   */
  bool InitInternal() override;

 private:
  bool InitSharedData();

  // bool init_synchronizer(const ModelConfig& config);
  bool InitPreprocessor();

  bool InitHdmap();

  bool AddDataAndPublishEvent(const std::shared_ptr<ImageLights> &data,
                              const CameraId &camera_id, double timestamp);

  // @brief sub long focus camera
  void SubLongFocusCamera(const sensor_msgs::Image &msg);

  // @brief sub short focus camera
  void SubShortFocusCamera(const sensor_msgs::Image &msg);

  void SubCameraImage(boost::shared_ptr<const sensor_msgs::Image> msg,
                      CameraId camera_id);

  void CameraSelection(double ts);
  bool VerifyLightsProjection(std::shared_ptr<ImageLights> image_lights);
  bool GetSignals(double ts, CarPose *pose,
                  std::vector<apollo::hdmap::Signal> *signals);
  bool GetCarPose(const double ts, CarPose *pose);

 private:
  TLPreprocessor preprocessor_;
  TLPreprocessingData *preprocessing_data_ = nullptr;

  HDMapInput *hd_map_ = nullptr;

  // signals
  float last_signals_ts_ = -1.0;
  std::vector<apollo::hdmap::Signal> last_signals_;
  float valid_hdmap_interval_ = 1.5;

  // tf
  double last_query_tf_ts_ = 0.0;
  float query_tf_inverval_seconds_ = 0.0;

  // process
  double last_proc_image_ts_ = 0.0;
  float proc_interval_seconds_ = 0.0;

  DISALLOW_COPY_AND_ASSIGN(TLPreprocessorSubnode);
};

REGISTER_SUBNODE(TLPreprocessorSubnode);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TL_PREPROCESSOR_SUBNODE_H_
