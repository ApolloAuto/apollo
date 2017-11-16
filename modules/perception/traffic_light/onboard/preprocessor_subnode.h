// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/09 13:39:42
// @file: preprocessor_subnode.h
// @brief: preprocessor_subnode is to sync 2-ways image &
//         push them down-stream.
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_SUBNODE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_SUBNODE_H

#include <memory>
#include <vector>
#include <deque>
#include <map>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber.h>
#include "modules/perception/traffic_light/onboard/hdmap_input.h"

#include "modules/perception/lib/base/timer.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/traffic_light/onboard/tl_shared_data.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/preprocessor/tl_preprocessor.h"
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

namespace apollo {
namespace perception {
namespace onboard {
class HDMapInput;
}  // namepace onboard

namespace traffic_light {

class TLPreprocessorSubnode;

//@brief pre-processor subnode
class TLPreprocessorSubnode : public Subnode {
 public:
  TLPreprocessorSubnode() = default;
  virtual ~TLPreprocessorSubnode() = default;;

  // @brief: as a subnode with type SUBNODE_IN
  //         we will use ros callback, so ignore subnode callback
  virtual StatusCode ProcEvents() override {
    return SUCC;
  }

  // for check lights projection on image border region dynamically
  static std::map<int, int> _s_image_borders;

 protected:
  //@brief init pre-processor
  virtual bool InitInternal() override;

 private:
  bool init_shared_data();

  //bool init_synchronizer(const ModelConfig& config);
  bool init_preprocessor();

  bool init_hdmap();

  bool add_data_and_publish_event(
      const std::shared_ptr<ImageLights> &data,
      const CameraId &camera_id,
      double timestamp);

  //@brief sub long focus camera
  void sub_long_focus_camera(const sensor_msgs::Image &msg);

  //@brief sub short focus camera
  void sub_short_focus_camera(const sensor_msgs::Image &msg);

  void sub_camera_image(const std::shared_ptr<sensor_msgs::Image> msg, CameraId camera_id);

  bool get_car_pose(const double ts, CarPose *pose);

  bool verify_lights_projection(
      const double &ts,
      const CameraId &camera_id,
      std::shared_ptr<ImageLights> *image_lights);

  // 原 sub_tf 的处理流程
  void add_cached_camera_selection(double timestamp);

 private:
  std::unique_ptr<TLPreprocessor> _preprocessor;
  MultiCamerasProjection _projection;

  //std::string                                                  _work_root_dir;
  //StreamInput<sensor_msgs::Image,
  //                     TLPreprocessorSubnode>                  _stream_input;
  image_transport::Subscriber sub_short_;
  image_transport::Subscriber sub_long_;
  TLPreprocessingData *_preprocessing_data = nullptr;

  HDMapInput *_hd_map = nullptr;  // HDMap

  std::map<int, double> _last_sub_camera_image_ts;
  std::map<int, std::string> _camera_topic_names;

  double _last_query_tf_ts = 0.0;
  double _query_tf_inverval_seconds = 0.0;

  double _last_proc_image_ts = 0.0;
  int _max_process_image_fps = 10;  // max frames to be processed per second
  double _proc_interval_seconds = 0.0;  //

  static std::map<int, std::string> _s_camera_names;
  static std::map<int, int> _s_camera_ts_last_3_digits;

 DISALLOW_COPY_AND_ASSIGN(TLPreprocessorSubnode);
};

REGISTER_SUBNODE(TLPreprocessorSubnode);
} // namespace traffic_light
} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_SUBNODE_H
