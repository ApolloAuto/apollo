// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/08/08
// @file: proc_subnode.cpp
// @brief: 
// 
#include "modules/perception/traffic_light/onboard/proc_subnode.h"
#include "modules/common/log.h"
#include <traffic_light/base/utils.h>
#include <traffic_light/rectify/unity/crop/cropbox.h>
#include "ctime"

//#include "roslibmetric/metric_handle.h"
#include "modules/perception/traffic_light/onboard/rosmetrichandle_singleton.h"
#include "lib/base/perf.h"
#include "lib/config_manager/config_manager.h"
#include "onboard/subnode_helper.h"
#include "onboard/shared_data_manager.h"
#include "modules/perception/traffic_light/onboard/proc_data.h"
#include "modules/perception/traffic_light/onboard/preprocessor_data.h"
#include "modules/perception/traffic_light/onboard/preprocessor_subnode.h"

namespace adu {
namespace perception {
namespace traffic_light {

using ::adu::perception::base::Singleton;

DEFINE_string(traffic_light_rectifier,
"",
"the rectifier enabled for traffic_light");
DEFINE_string(traffic_light_recognizer,
"",
"the recognizer enabled for traffic_light");
DEFINE_string(traffic_light_reviser,
"", "the reviser enabled for traffic_light");

DEFINE_double(valid_ts_interval,
100,
"the difference between event ts and now ts must be less than me.Unit:second ");

TLProcSubnode::~TLProcSubnode() {
  _preprocessing_data = nullptr;
  _proc_data = nullptr;
}

bool TLProcSubnode::init_internal() {

  if (!init_shared_data()) {
    AERROR << "TLProcSubnode init shared data failed.";
    return false;
  }
  if (!init_rectifier()) {
    AERROR << "TLProcSubnode init rectifier failed.";
    return false;
  }
  if (!init_recognizer()) {
    AERROR << "TLProcSubnode init recognizer failed.";
    return false;
  }
  if (!init_reviser()) {
    AERROR << "TLProcSubnode init reviser failed.";
    return false;
  }

  // init image_border
  config_manager::ConfigManager *config_manager
      = base::Singleton<config_manager::ConfigManager>::get();
  std::string model_name("TLProcSubnode");
  const config_manager::ModelConfig *model_config(nullptr);
  if (!config_manager->get_model_config(model_name, &model_config)) {
    AERROR << "TLProcSubnode not found model: " << model_name;
    return false;
  }
  if (!model_config->get_value("image_border",
                               &_image_border)) {
    AERROR << "TLProcSubnode Failed to find Conf: "
           << "image_border.";
    return false;
  }
  int crop_method = 0;
  switch (crop_method) {
    default:
    case 0: {
      float crop_scale = 0;
      float crop_min_size = 0;
      _crop.reset(new CropBox(crop_scale, crop_min_size));
    }
      break;
    case 1:_crop.reset(new CropBoxWholeImage());
      break;
  }

  AINFO << "TLProcSubnode init successfully. ";
  return true;
}

bool TLProcSubnode::handle_event(const onboard::Event &sub_event,
                                 onboard::Event *pub_event) {
  const double proc_subnode_handle_event_start_ts = base::TimeUtil::get_current_time();
  PERF_FUNCTION();
  // get up-stream data
  const double timestamp = sub_event.timestamp;
  const std::string device_id = sub_event.reserve;
  pub_event->local_timestamp = base::TimeUtil::get_current_time();

  AINFO << "Detect Start ts:" << GLOG_TIMESTAMP(timestamp);
  std::string key;
  if (!onboard::SubnodeHelper::produce_shared_data_key(timestamp, device_id,
                                                       &key)) {
    AERROR << "TLProcSubnode produce_shared_data_key failed."
           << " ts:" << timestamp << " device_id:" << device_id;
    return false;
  }

  onboard::SharedDataPtr<ImageLights> image_lights;
  if (!_preprocessing_data->get(key, &image_lights)) {
    AERROR << "TLProcSubnode failed to get shared data,"
           << " name:" << _preprocessing_data->name()
           << ", time: " << GLOG_TIMESTAMP(timestamp);
    return false;
  }
  AINFO << "TLProcSubnode get shared data ok,ts: " << GLOG_TIMESTAMP(timestamp);

  // preprocess send a msg -> proc receive a msg
  double enter_proc_latency = (proc_subnode_handle_event_start_ts -
      image_lights->preprocess_send_timestamp);

  if (base::TimeUtil::get_current_time() - sub_event.local_timestamp > FLAGS_valid_ts_interval) {
    AERROR << "TLProcSubnode failed to process image"
           << "Because images are too old"
           << ",current time: " << GLOG_TIMESTAMP(base::TimeUtil::get_current_time())
           << ", event time: " << GLOG_TIMESTAMP(sub_event.local_timestamp);
    return false;
  }

  // verify image_lights from cameras
  RectifyOption rectify_option;
  if (!verify_image_lights(*image_lights, &rectify_option.camera_id)) {
    AERROR << "TLProcSubnode invalid image_lights ";
    return false;
  }

  //cv::Rect cbox;
  //_crop->get_crop_box(image_lights->image->size(), *(image_lights->lights), &cbox);
  if (!image_lights->image->generate_mat()) {
    AERROR << "TLProcSubnode failed to generate mat";
    return false;
  }
  // using rectifier to rectify the region.
  const double before_rectify_ts = base::TimeUtil::get_current_time();
  if (!_rectifier->rectify(*(image_lights->image), rectify_option,
                           (image_lights->lights).get())) {
    AERROR << "TLProcSubnode failed to rectify the regions "
           << "ts:" << GLOG_TIMESTAMP(timestamp) << " Image:" << *(image_lights->image);
    return false;
  }
  const double detection_latency = base::TimeUtil::get_current_time() - before_rectify_ts;

  // update image_border
  const double before_update_image_border_ts = base::TimeUtil::get_current_time();
  base::MutexLock lock(&_mutex);
  int cam_id = static_cast<int>(image_lights->camera_id);
  compute_image_border(*image_lights,
                       &TLPreprocessorSubnode::_s_image_borders[cam_id]);
  AINFO << "TLProcSubnode update image_border size: "
        << TLPreprocessorSubnode::_s_image_borders[cam_id]
        << " ts: " << GLOG_TIMESTAMP(timestamp)
        << " CameraId: " << image_lights->camera_id;
  image_lights->offset = TLPreprocessorSubnode::_s_image_borders[cam_id];
  const double update_image_border_latency =
      base::TimeUtil::get_current_time() - before_update_image_border_ts;

  // 给 CarOS Monitor 发异常
  // 投影框偏移较大
  if (image_lights->offset > 150) {
    RosMetricHandleSingleton *ros_mh = Singleton<RosMetricHandleSingleton>::get();
    // std::string debug_string = "Offset between projection_box and detection_box(in pixel): " +
    //         std::to_string(image_lights->offset) + ". Check camera extrinsics.";
    std::string debug_string = "";
    debug_string += ("offset:" + std::to_string(image_lights->offset));
    debug_string += (",camera_id:" + CAMERA_ID_TO_STR.at(image_lights->camera_id));
    ros_mh->throw_exception(1, 102, debug_string);
    XLOG(WARN) << "Offset between projection_box and detection_box(in pixel): "
               << std::to_string(image_lights->offset)
               << ". Check camera extrinsics"
               << ". debug_string: " << debug_string;
  }

  // recognize_status
  const double before_recognization_ts = base::TimeUtil::get_current_time();
  if (!_recognizer->recognize_status(*(image_lights->image), RecognizeOption(),
                                     (image_lights->lights).get())) {
    AERROR << "TLProcSubnode failed to recognize lights,"
           << " ts:" << GLOG_TIMESTAMP(timestamp)
           << " image:" << image_lights->image;
    return false;
  }
  const double recognization_latency =
      base::TimeUtil::get_current_time() - before_recognization_ts;

  // revise status
  const double before_revise_ts = base::TimeUtil::get_current_time();
  if (!_reviser->revise(ReviseOption(sub_event.timestamp), image_lights->lights.get())) {
    AERROR << "TLReviserSubnode revise data failed. "
           << "sub_event:" << sub_event.to_string();
    return false;
  }
  const double revise_latency = base::TimeUtil::get_current_time() - before_revise_ts;

  AINFO << "TLProcSubnode process traffic_light, "
        << " msg_ts: " << GLOG_TIMESTAMP(timestamp)
        << " from device_id: " << device_id
        << " get " << image_lights->lights->size() << " lights."
        << " detection_latency: " << detection_latency * 1000 << " ms."
        << " recognization_latency: " << recognization_latency * 1000 << " ms."
        << " revise_latency: " << revise_latency * 1000 << " ms."
        << " TLProcSubnode::handle_event latency: "
        << (base::TimeUtil::get_current_time() -
            proc_subnode_handle_event_start_ts) * 1000 << " ms."
        << " enter_proc_latency: " << enter_proc_latency * 1000 << " ms."
        << " preprocess_latency: " << (image_lights->preprocess_send_timestamp -
      image_lights->preprocess_receive_timestamp) * 1000
        << " ms.";
  // }

  // add to down-stream data
  if (!_proc_data->add(key, image_lights)) {
    AERROR << "TLProcSubnode failed to add data down-stream, "
           << " key:" << key;
    return false;
  }

  // set pub_event
  pub_event->timestamp = timestamp;
  pub_event->reserve = device_id;

  return true;
}

bool TLProcSubnode::init_shared_data() {
  CHECK_NOTNULL(_shared_data_manager);

  const std::string preprocessing_data_name("TLPreprocessingData");
  _preprocessing_data = dynamic_cast<TLPreprocessingData *>(
      _shared_data_manager->get_shared_data(preprocessing_data_name));
  if (_preprocessing_data == nullptr) {
    AERROR << "TLProcSubnode failed to get shared data instance: "
           << preprocessing_data_name;
    return false;
  }

  const std::string proc_data_name("TLProcData");
  _proc_data = dynamic_cast<TLProcData *>(
      _shared_data_manager->get_shared_data(proc_data_name));
  if (_proc_data == nullptr) {
    AERROR << "Failed to get shared data instance: "
           << proc_data_name;
    return false;
  }

  AINFO << "Init shared data successfully, "
        << "preprocessing_data: " << _preprocessing_data->name()
        << "proc_data:" << _proc_data->name();
  return true;
}

bool TLProcSubnode::init_rectifier() {

  _rectifier.reset(BaseRectifierRegisterer::get_instance_by_name(
      FLAGS_traffic_light_rectifier));
  if (!_rectifier) {
    AERROR << "TLProcSubnode new rectifier failed. rectifier name:"
           << FLAGS_traffic_light_rectifier << " failed.";
    return false;
  }
  if (!_rectifier->init()) {
    AERROR << "TLProcSubnode init rectifier failed. rectifier name:"
           << FLAGS_traffic_light_rectifier << " failed.";
    return false;
  }
  return true;
}

bool TLProcSubnode::init_recognizer() {
  _recognizer.reset(BaseRecognizerRegisterer::get_instance_by_name(
      FLAGS_traffic_light_recognizer));
  if (!_recognizer) {
    AERROR << "TLProcSubnode new recognizer failed. name:"
           << FLAGS_traffic_light_recognizer;
    return false;
  }
  if (!_recognizer->init()) {
    AERROR << "TLProcSubnode init recognizer failed.";
    return false;
  }
  return true;
}

bool TLProcSubnode::init_reviser() {
  _reviser.reset(BaseReviserRegisterer::get_instance_by_name(
      FLAGS_traffic_light_reviser));
  if (_reviser == nullptr) {
    AERROR << "TLProcSubnode new reviser failed. name:"
           << FLAGS_traffic_light_reviser;
    return false;
  }
  if (!_reviser->init()) {
    AERROR << "TLProcSubnode init reviser failed. name:"
           << FLAGS_traffic_light_reviser;
    return false;
  }
  return true;
}

double TLProcSubnode::get_mean_distance(const double ts,
                                        const Eigen::Matrix4d &car_pose,
                                        const LightPtrs &lights) const {
  if (lights.empty()) {
    XLOG(WARN) << "get_mean_distance failed. lights is empty, "
               << "while it should not be. ts:" << GLOG_TIMESTAMP(ts);
    return DBL_MAX;
  }

  double distance = 0.0;
  for (const LightPtr &light : lights) {
    auto light_distance = stopline_distance(car_pose, light->info.stop_line());
    if (light_distance < 0) {
      XLOG(WARN) << "get_mean_distance failed. lights stop line data is illegal, "
                 << "ts:" << GLOG_TIMESTAMP(ts);
      return DBL_MAX;
    }
    distance += light_distance;
  }
  return distance / lights.size();
}

bool TLProcSubnode::verify_image_lights(
    const ImageLights &image_lights, CameraId *selection) const {
  if (!image_lights.image || !image_lights.image->contain_image()) {
    AERROR << "TLProcSubnode image_lights has no image, "
           << "verify_image_lights failed.";
    return false;
  }

  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  const int cam_id = static_cast<int>(image_lights.camera_id);
  if (cam_id < 0 || cam_id >= num_camera_ids) {
    AERROR << "TLProcSubnode image_lights unknown camera id, "
           << "verify_image_lights failed.";
    return false;
  }
  for (LightPtr light:*(image_lights.lights)) {
    if (!box_is_valid(light->region.projection_roi, image_lights.image->size())) {
      clear_box(light->region.projection_roi);
      continue;
    }
  }
  *selection = image_lights.camera_id;

  return true;
}

bool TLProcSubnode::compute_image_border(const ImageLights &image_lights,
                                         int *image_border) {
  if (!image_lights.image) {
    AERROR << "TLProcSubnode image_lights has no image, "
           << "compute_image_border failed.";
    return false;
  }

  auto camera_id = static_cast<int>(image_lights.camera_id);
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  if (camera_id < 0 || camera_id >= num_camera_ids) {
    AERROR << "TLProcSubnode image_lights unknown camera selection, "
           << "compute_image_border failed, "
           << "camera_id: " << CAMERA_ID_TO_STR.at(image_lights.camera_id);
    return false;
  }

  // check lights info
  if (image_lights.lights->empty()) {
    AINFO << "TLProcSubnode image_lights no lights info, "
          << "no need to update image border, reset image border size to 100";
    *image_border = 100;
    return true;
  }

  if (camera_id == static_cast<int>(CameraId::UNKNOWN) - 1) {
    AINFO << "TLProcSubnode no need to update image border, "
          << "camera_id: " << CAMERA_ID_TO_STR.at(image_lights.camera_id);
    return true;
  }

  LightPtrs &lights_ref = *(image_lights.lights.get());
  int max_offset = -1;
  for (size_t i = 0; i < lights_ref.size(); ++i) {
    cv::Rect rectified_roi = lights_ref[i]->region.rectified_roi;
    cv::Rect projection_roi = lights_ref[i]->region.projection_roi;
    // 有多个灯，取最大偏移
    int offset = 0;
    compute_rects_offset(projection_roi, rectified_roi, &offset);
    max_offset = std::max(max_offset, offset);
  }
  if (max_offset != -1) {
    *image_border = max_offset;
  }

  return true;
}

void TLProcSubnode::compute_rects_offset(
    const cv::Rect &rect1,
    const cv::Rect &rect2,
    int *offset) {
  cv::Point center1(rect1.x + rect1.width / 2, rect1.y + rect1.height / 2);
  cv::Point center2(rect2.x + rect2.width / 2, rect2.y + rect2.height / 2);

  cv::Point pt1;
  cv::Point pt2;
  TLPreprocessorSubnode::_s_debug_roi_relative_pos = "unknown";
  // 分四个象限, 记录横、纵方向最大偏移
  if (center2.y <= center1.y) {
    if (center2.x >= center1.x) {
      pt1 = cv::Point(rect1.x + rect1.width, rect1.y);
      pt2 = cv::Point(rect2.x + rect2.width, rect2.y);
      TLPreprocessorSubnode::_s_debug_roi_relative_pos = "top right";
    } else {
      pt1 = cv::Point(rect1.x, rect1.y);
      pt2 = cv::Point(rect2.x, rect2.y);
      TLPreprocessorSubnode::_s_debug_roi_relative_pos = "top left";
    }
  } else {
    if (center2.x >= center1.x) {
      pt1 = cv::Point(rect1.x + rect1.width, rect1.y + rect1.height);
      pt2 = cv::Point(rect2.x + rect2.width, rect2.y + rect2.height);
      TLPreprocessorSubnode::_s_debug_roi_relative_pos = "bottom right";
    } else {
      pt1 = cv::Point(rect1.x, rect1.y + rect1.height);
      pt2 = cv::Point(rect2.x, rect2.y + rect2.height);
      TLPreprocessorSubnode::_s_debug_roi_relative_pos = "bottom left";
    }
  }

  *offset = std::max(abs(pt1.x - pt2.x), abs(pt1.y - pt2.y));
}

REGISTER_SUBNODE(TLProcSubnode);

} // namespace traffic_light
} // namespace perception
} // namespace adu
