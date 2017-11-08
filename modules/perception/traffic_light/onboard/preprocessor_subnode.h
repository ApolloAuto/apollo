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
#include <onboard/transform_input_wrapper.h>

#include "lib/base/perf.h"
#include "onboard/subnode.h"
#include "onboard/stream_input.h"
#include "onboard/subnode_helper.h"
#include "module/perception/traffic_light/onboard/preprocessor_data.h"
#include "module/perception/traffic_light/base/image.h"
#include "module/perception/traffic_light/preprocessor/tl_preprocessor.h"
#include "module/perception/traffic_light/projection/multi_camera_projection.h"

namespace adu {
namespace perception {
namespace onboard {
class HDMapInput;
}  // namepace onboard

namespace traffic_light {

class TLPreprocessorSubnode;

//@brief pre-processor subnode
class TLPreprocessorSubnode : public onboard::Subnode {
 public:
  TLPreprocessorSubnode();
  virtual ~TLPreprocessorSubnode();

  // @brief: as a subnode with type SUBNODE_IN
  //         we will use ros callback, so ignore subnode callback
  virtual onboard::Status proc_events() override {
    return onboard::SUCC;
  }

  // for check lights projection on image border region dynamically
  static std::map<int, int> _s_image_borders;
  static std::string _s_debug_roi_relative_pos;  // for check relative

 protected:
  //@brief init pre-processor
  virtual bool init_internal() override;

 private:
  bool init_shared_data();

  //bool init_synchronizer(const config_manager::ModelConfig& config);
  bool init_preprocessor();

  bool init_transform_input(const std::map<std::string, std::string> &fields);
  bool init_hdmap();

  bool init_subscriber(const std::map<std::string, std::string> &fields,
                       const CameraId &camera_id,
                       void (TLPreprocessorSubnode::*)(const sensor_msgs::ImageConstPtr &));

  bool add_data_and_publish_event(
      const std::shared_ptr<ImageLights> &data,
      const CameraId &camera_id,
      double timestamp);

  //@brief sub long focus camera
  void sub_long_focus_camera(const sensor_msgs::ImageConstPtr &msg);

  //@brief sub short focus camera
  void sub_short_focus_camera(const sensor_msgs::ImageConstPtr &msg);

  void sub_2mm_focus_camera(const sensor_msgs::ImageConstPtr &msg);

  void sub_12mm_focus_camera(const sensor_msgs::ImageConstPtr &msg);

  void sub_camera_image(const sensor_msgs::ImageConstPtr &msg, CameraId camera_id);

  //@brief ros msg to image.
  bool rosmsg_to_image(const CameraId camera_id, const sensor_msgs::ImageConstPtr &msg,
                       Image *image);

  //@brief ros msg to cv::Mat
  bool rosmsg_to_cv_mat(const sensor_msgs::ImageConstPtr &msg,
                        cv::Mat *mat);

  //@brief init tf subscriber
  bool init_tf_subscriber(
      void (TLPreprocessorSubnode::*)(const tf2_msgs::TFMessageConstPtr &));

  //@brief tf subscribe callback
  //void sub_tf(const tf::tfMessage::ConstPtr&);
  void sub_tf(const tf2_msgs::TFMessageConstPtr &);

  //@brief parse tfMessage's timestamp
  bool parse_tf_msg_timestamp(const tf2_msgs::TFMessageConstPtr &msg, double *timestamp);

  //@brief parse tfMessage's child_frame_id
  bool parse_tf_msg_child_frame_id(const tf2_msgs::TFMessageConstPtr &msg,
                                   std::string *child_frame_id);

  bool get_car_pose(const double ts, CarPose *pose);

  bool verify_lights_projection(
      const double &ts,
      const CameraId &camera_id,
      std::shared_ptr<ImageLights> *image_lights);

 private:
  std::unique_ptr<TLPreprocessor> _preprocessor;
  MultiCamerasProjection _projection;

  std::string _work_root_dir;
  onboard::StreamInput<sensor_msgs::Image,
                       TLPreprocessorSubnode> _stream_input;
  TLPreprocessingData *_preprocessing_data = nullptr;

  onboard::StreamInput<tf2_msgs::TFMessage, TLPreprocessorSubnode> _tf_stream_input;
  onboard::TransformInputWrapper _velodyne2world_trans; // TF module
  onboard::HDMapInput *_hd_map = nullptr;  // HDMap

  std::map<int, double> _last_sub_camera_image_ts;
  std::map<int, std::string> _camera_topic_names;

  double _last_sub_tf_ts = 0.0;
  double _sub_tf_inverval_seconds = 0.0;

  double _last_proc_image_ts = 0.0;
  int _max_process_image_fps = 10;  // max frames to be processed per second
  double _proc_interval_seconds = 0.0;  //

  static std::map<int, std::string> _s_camera_names;
  static std::map<int, int> _s_camera_ts_last_3_digits;

  DISALLOW_COPY_AND_ASSIGN(TLPreprocessorSubnode);
};

} // namespace traffic_light
} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_SUBNODE_H
