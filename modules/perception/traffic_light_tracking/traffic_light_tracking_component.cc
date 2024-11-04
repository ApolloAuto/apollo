/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/traffic_light_tracking/traffic_light_tracking_component.h"

#include <limits>
#include <map>
#include <utility>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "modules/perception/traffic_light_tracking/proto/traffic_light_tracking_component.pb.h"

#include "cyber/common/file.h"
#include "cyber/profiler/profiler.h"
#include "cyber/time/clock.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/common/onboard/common_flags/common_flags.h"
#include "modules/perception/traffic_light_tracking/interface/base_traffic_light_tracker.h"

namespace apollo {
namespace perception {
namespace trafficlight {

using apollo::perception::onboard::FLAGS_start_visualizer;
using TLCamID = apollo::perception::TrafficLightDetection::CameraID;
using apollo::cyber::Clock;

class TLInfo {
 public:
  cv::Scalar tl_color_;
  std::string tl_string_;
  std::string tl_string_ex_;
};

std::map<base::TLColor, TLInfo> s_tl_infos = {
    {base::TLColor::TL_UNKNOWN_COLOR,
     {cv::Scalar(255, 255, 255), "UNKNOWN", "UNKNOWN traffic light"}},
    {base::TLColor::TL_RED,
     {cv::Scalar(0, 0, 255), "RED", "RED traffic light"}},
    {base::TLColor::TL_GREEN,
     {cv::Scalar(0, 255, 0), "GREEN", "GREEN traffic light"}},
    {base::TLColor::TL_YELLOW,
     {cv::Scalar(0, 255, 255), "YELLOW", "YELLOW traffic light"}}};

bool TrafficLightTrackComponent::Init() {
  // init component config
  if (InitConfig() != cyber::SUCC) {
    AERROR << "TrafficLightTrackComponent InitConfig failed.";
    return false;
  }
  // init algorithm plugin
  if (InitAlgorithmPlugin() != cyber::SUCC) {
    AERROR << "TrafficLightTrackComponent InitAlgorithmPlugin failed.";
    return false;
  }

  if (InitV2XListener() != cyber::SUCC) {
    AERROR << "TrafficLightTrackComponent InitV2XListener failed.";
    return false;
  }

  return true;
}

bool TrafficLightTrackComponent::Proc(
    const std::shared_ptr<TrafficDetectMessage>& message) {
  PERF_FUNCTION()
  auto time_imags = std::to_string(message->timestamp_);
  AINFO << "Enter tracking component, message timestamp: " << time_imags;

  bool status = InternalProc(message);

  return status;
}

int TrafficLightTrackComponent::InitConfig() {
  apollo::perception::trafficlight::TrackingParam traffic_light_param;
  if (!GetProtoConfig(&traffic_light_param)) {
    AINFO << "load trafficlights tracking component proto param failed";
    return cyber::FAIL;
  }

  v2x_trafficlights_input_channel_name_ =
      traffic_light_param.v2x_trafficlights_input_channel_name();

  auto plugin_param = traffic_light_param.plugin_param();
  tl_tracker_name_ = plugin_param.name();
  config_path_ = plugin_param.config_path();
  config_file_ = plugin_param.config_file();
  AINFO << "tl_tracker_name: " << tl_tracker_name_
        << " config_path: " << config_path_ << " config_file: " << config_file_;

  writer_ = node_->CreateWriter<apollo::perception::TrafficLightDetection>(
      traffic_light_param.traffic_light_output_channel_name());
  v2x_msg_buffer_.set_capacity(max_v2x_msg_buff_size_);

  return cyber::SUCC;
}

bool TrafficLightTrackComponent::InitAlgorithmPlugin() {
  trafficlight::BaseTrafficLightTracker* tracker =
      trafficlight::BaseTrafficLightTrackerRegisterer::GetInstanceByName(
          tl_tracker_name_);
  CHECK_NOTNULL(tracker);
  tracker_.reset(tracker);
  ACHECK(tracker_ != nullptr);

  tracker_init_options_.config_path = config_path_;
  tracker_init_options_.config_file = config_file_;

  if (!tracker_->Init(tracker_init_options_)) {
    AERROR << "Trafficlight tracking init failed";
    return false;
  }

  return cyber::SUCC;
}

int TrafficLightTrackComponent::InitV2XListener() {
  typedef const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData>
      V2XTrafficLightsMsgType;
  std::function<void(const V2XTrafficLightsMsgType&)> sub_v2x_tl_callback =
      std::bind(&TrafficLightTrackComponent::OnReceiveV2XMsg, this,
                std::placeholders::_1);
  auto sub_v2x_reader = node_->CreateReader(
      v2x_trafficlights_input_channel_name_, sub_v2x_tl_callback);
  return cyber::SUCC;
}

bool TrafficLightTrackComponent::InternalProc(
    const std::shared_ptr<TrafficDetectMessage const>& message) {
  PERF_BLOCK("traffic_light_tracking")
  bool status = tracker_->Track(message->traffic_light_frame_.get());
  PERF_BLOCK_END

  auto frame = message->traffic_light_frame_;
  AINFO << "Enter SyncV2XTrafficLights founction.";
  SyncV2XTrafficLights(frame.get());
  stoplines_ = message->stoplines_;

  std::shared_ptr<TrafficLightDetection> out_msg(new TrafficLightDetection);
  auto& camera_name = frame->data_provider->sensor_name();
  AINFO << "Enter TransformOutputMessage founction.";
  if (!TransformOutputMessage(frame.get(), camera_name, &out_msg, message)) {
    AERROR << "transform_output_message failed, msg_time: "
           << message->timestamp_;
    return false;
  }

  // send msg
  writer_->Write(out_msg);
  AINFO << "Send trafficlight tracking output message.";

  return true;
}

void TrafficLightTrackComponent::OnReceiveV2XMsg(
    const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData> v2x_msg) {
  std::lock_guard<std::mutex> lck(mutex_);
  v2x_msg_buffer_.push_back(*v2x_msg);
}

void TrafficLightTrackComponent::SyncV2XTrafficLights(
    camera::TrafficLightFrame* frame) {
  const double camera_frame_timestamp = frame->timestamp;
  auto sync_single_light = [&](base::TrafficLightPtr light) {
    for (auto itr = v2x_msg_buffer_.rbegin(); itr != v2x_msg_buffer_.rend();
         ++itr) {
      double v2x_timestamp = itr->header().timestamp_sec();
      // find close enough v2x msg
      if (std::fabs(camera_frame_timestamp - v2x_timestamp) <
          v2x_sync_interval_seconds_) {
        const int v2x_lights_num =
            itr->road_traffic_light(0).single_traffic_light_size();
        const auto& v2x_lights = itr->road_traffic_light(0);
        for (int i = 0; i < v2x_lights_num; ++i) {
          const auto& v2x_light = v2x_lights.single_traffic_light(i);
          // check signal id
          if (light->id != v2x_light.id()) {
            continue;
          }
          base::TLColor v2x_color = base::TLColor::TL_UNKNOWN_COLOR;
          bool blink = false;
          switch (v2x_light.color()) {
            default:
            case apollo::v2x::SingleTrafficLight::UNKNOWN:
              v2x_color = base::TLColor::TL_UNKNOWN_COLOR;
              break;
            case apollo::v2x::SingleTrafficLight::RED:
              v2x_color = base::TLColor::TL_RED;
              break;
            case apollo::v2x::SingleTrafficLight::YELLOW:
              v2x_color = base::TLColor::TL_YELLOW;
              break;
            case apollo::v2x::SingleTrafficLight::GREEN:
              v2x_color = base::TLColor::TL_GREEN;
              break;
            case apollo::v2x::SingleTrafficLight::BLACK:
              v2x_color = base::TLColor::TL_BLACK;
              break;
            case apollo::v2x::SingleTrafficLight::FLASH_GREEN:
              v2x_color = base::TLColor::TL_GREEN;
              blink = true;
              break;
          }
          // use v2x result directly
          AINFO << "Sync V2X success. update color from "
                << static_cast<int>(light->status.color) << " to "
                << static_cast<int>(v2x_color) << "; signal id: " << light->id;
          light->status.color = v2x_color;
          light->status.blink = blink;
        }
        break;
      }
    }
  };
  for (auto& light : frame->traffic_lights) {
    sync_single_light(light);
  }
}

double TrafficLightTrackComponent::stopline_distance(
    const Eigen::Matrix4d& cam_pose) {
  if (stoplines_.empty()) {
    AWARN << "compute car to stopline's distance failed(no stopline). "
          << "cam_pose:" << cam_pose;
    return -1;
  }
  const apollo::hdmap::Curve& stopline = stoplines_.Get(0);
  if (stopline.segment().empty()) {
    AWARN << "compute car to stopline's distance"
          << " failed(stopline has no segment line). "
          << "cam_pose:" << cam_pose
          << " stopline:" << stopline.ShortDebugString();
    return -1;
  }
  if (!stopline.segment(0).has_line_segment()) {
    AWARN << "compute car to stopline's distance "
          << "failed(stopline has no segment). "
          << "cam_pose:" << cam_pose
          << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  if (stopline.segment(0).line_segment().point().empty()) {
    AWARN << "compute car to stopline's distance "
          << "failed(stopline has no point). "
          << "cam_pose:" << cam_pose
          << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  Eigen::Vector3d stopline_pt(stopline.segment(0).line_segment().point(0).x(),
                              stopline.segment(0).line_segment().point(0).y(),
                              stopline.segment(0).line_segment().point(0).z());
  Eigen::Vector3d stopline_pt_cam =
      (cam_pose.inverse() *
       Eigen::Vector4d(stopline_pt(0), stopline_pt(1), stopline_pt(2), 1.0))
          .head(3);

  return stopline_pt_cam(2);
}

bool TrafficLightTrackComponent::TransformOutputMessage(
    camera::TrafficLightFrame* frame, const std::string& camera_name,
    std::shared_ptr<TrafficLightDetection>* out_msg,
    const std::shared_ptr<TrafficDetectMessage const>& message) {
  auto& lights = frame->traffic_lights;
  auto* header = (*out_msg)->mutable_header();
  double publish_time = Clock::NowInSeconds();
  header->set_timestamp_sec(publish_time);  // message publishing time

  // Set traffic light color to unknown before the process
  detected_trafficlight_color_ = base::TLColor::TL_UNKNOWN_COLOR;

  // sec -> nano-sec
  uint64_t ts_int64 = static_cast<uint64_t>(frame->timestamp * 1e9);
  header->set_camera_timestamp(ts_int64);

  (*out_msg)->set_camera_name(camera_name);

  // Do voting from multiple traffic light detections
  cnt_r_ = 0;
  int max_r_id = -1;
  double max_r_conf = 0;

  cnt_g_ = 0;
  int max_g_id = -1;
  double max_g_conf = 0;

  cnt_y_ = 0;
  int max_y_id = -1;
  double max_y_conf = 0;

  cnt_u_ = 0;

  int max_n_id = -1;

  for (int i = 0; i < static_cast<int>(lights.size()); i++) {
    switch (lights.at(i)->status.color) {
      case base::TLColor::TL_RED:
        // quick fix for 0 confidence color decision
        if (std::abs(lights.at(i)->status.confidence) <
            std::numeric_limits<double>::min()) {
          lights.at(i)->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_n_id = i;
          break;
        }
        cnt_r_ += lights.at(i)->status.confidence;
        if (lights.at(i)->status.confidence >= max_r_conf) {
          max_r_id = i;
          max_r_conf = lights.at(i)->status.confidence;
        }
        break;
      case base::TLColor::TL_GREEN:
        // quick fix for 0 confidence color decision
        if (std::abs(lights.at(i)->status.confidence) <
            std::numeric_limits<double>::min()) {
          lights.at(i)->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_n_id = i;
          break;
        }
        cnt_g_ += lights.at(i)->status.confidence;
        if (lights.at(i)->status.confidence >= max_g_conf) {
          max_g_id = i;
          max_g_conf = lights.at(i)->status.confidence;
        }
        break;
      case base::TLColor::TL_YELLOW:
        // quick fix for 0 confidence color decision
        if (std::abs(lights.at(i)->status.confidence) <
            std::numeric_limits<double>::min()) {
          lights.at(i)->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_n_id = i;
          break;
        }
        cnt_y_ += lights.at(i)->status.confidence;
        if (lights.at(i)->status.confidence >= max_y_conf) {
          max_y_id = i;
          max_y_conf = lights.at(i)->status.confidence;
        }
        break;
      case base::TLColor::TL_UNKNOWN_COLOR:
        cnt_u_ += lights.at(i)->status.confidence;
        max_n_id = i;
        break;
      default:
        max_n_id = i;
        break;
    }
  }

  int max_light_id = -1;
  if (cnt_r_ >= cnt_g_ && cnt_r_ >= cnt_y_ && cnt_r_ > 0) {
    max_light_id = max_r_id;
  } else if (cnt_y_ > cnt_r_ && cnt_y_ >= cnt_g_) {
    max_light_id = max_y_id;
  } else if (cnt_g_ > cnt_r_ && cnt_g_ > cnt_y_) {
    max_light_id = max_g_id;
  } else if (cnt_r_ == 0 && cnt_g_ == 0 && cnt_y_ == 0) {
    max_light_id = max_n_id;
  }

  // swap the final output light to the first place
  if (max_light_id > 0) {
    std::swap(lights[0], lights[max_light_id]);
  }

  if (max_light_id >= 0) {
    for (size_t i = 0; i < lights.size(); i++) {
      apollo::perception::TrafficLight* light_result =
          (*out_msg)->add_traffic_light();
      light_result->set_id(lights.at(i)->id);
      light_result->set_confidence(lights.at(0)->status.confidence);
      light_result->set_color(
          static_cast<apollo::perception::TrafficLight_Color>(
              lights.at(0)->status.color));
      light_result->set_blink(lights.at(0)->status.blink);
    }
    // set contain_lights
    (*out_msg)->set_contain_lights(lights.size() > 0);
    detected_trafficlight_color_ = lights.at(0)->status.color;
  }

  AINFO << "Enter TransformDebugMessage founction.";
  // add traffic light debug info
  if (!TransformDebugMessage(frame, out_msg, message)) {
    AERROR << "ProcComponent::Proc failed to transform debug msg.";
    return false;
  }

  return true;
}

bool TrafficLightTrackComponent::TransformDebugMessage(
    const camera::TrafficLightFrame* frame,
    std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg,
    const std::shared_ptr<TrafficDetectMessage const>& in_message) {
  const auto& lights = frame->traffic_lights;
  // add traffic light debug info
  TrafficLightDebug* light_debug = (*out_msg)->mutable_traffic_light_debug();

  // signal number
  light_debug->set_signal_num(static_cast<int>(lights.size()));

  if (!lights.empty() && !lights[0]->region.debug_roi.empty()) {
    const auto& debug_roi = lights[0]->region.debug_roi;
    // Crop ROI
    TransRect2Box(debug_roi[0], light_debug->mutable_cropbox());

    // debug ROI (candidate detection boxes)
    for (auto iter = debug_roi.begin() + 1; iter != debug_roi.end(); ++iter) {
      TransRect2Box(*iter, light_debug->add_box());
      TransRect2Box(*iter, light_debug->add_debug_roi());
    }
  }

  for (const auto& light : lights) {
    // Detection ROI
    auto* box = light_debug->add_box();
    TransRect2Box(light->region.detection_roi, box);
    box->set_color(static_cast<TrafficLight_Color>(light->status.color));
    box->set_selected(true);

    // Projection ROI
    TransRect2Box(light->region.projection_roi, light_debug->add_box());
    TransRect2Box(light->region.projection_roi,
                  light_debug->add_projected_roi());

    // Crop ROI
    TransRect2Box(light->region.debug_roi[0], light_debug->add_crop_roi());

    // Rectified ROI
    auto* rectified_roi = light_debug->add_rectified_roi();
    TransRect2Box(light->region.detection_roi, rectified_roi);
    rectified_roi->set_color(
        static_cast<TrafficLight_Color>(light->status.color));
    rectified_roi->set_selected(true);
  }

  if (lights.size() > 0) {
    auto& pose_ptr = in_message->carpose_;
    Eigen::Matrix4d cam_pose;
    auto& camera_name = frame->data_provider->sensor_name();
    cam_pose = pose_ptr->c2w_poses_.at(camera_name);
    light_debug->set_distance_to_stop_line(stopline_distance(cam_pose));
  }

  if (FLAGS_start_visualizer) {
    Visualize(*frame, lights);
  }

  return true;
}

void TrafficLightTrackComponent::TransRect2Box(
    const base::RectI& rect, apollo::perception::TrafficLightBox* box) {
  box->set_x(rect.x);
  box->set_y(rect.y);
  box->set_width(rect.width);
  box->set_height(rect.height);
}

void TrafficLightTrackComponent::Visualize(
    const camera::TrafficLightFrame& frame,
    const std::vector<base::TrafficLightPtr>& lights) const {
  char str[100];
  std::string tl_string;
  cv::Scalar tl_color;

  if (lights.empty()) {
    return;
  }
  cv::Mat output_image(image_height_, image_width_, CV_8UC3,
                       cv::Scalar(0, 0, 0));
  base::Image8U out_image(image_height_, image_width_, base::Color::RGB);
  camera::DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  frame.data_provider->GetImage(image_options, &out_image);
  memcpy(output_image.data, out_image.cpu_data(),
         out_image.total() * sizeof(uint8_t));

  for (const auto& light : lights) {
    // Crop ROI
    const auto& crop_roi = light->region.debug_roi[0];
    const cv::Rect rect_crop(crop_roi.x, crop_roi.y, crop_roi.width,
                             crop_roi.height);
    if (light == lights[0])
      cv::rectangle(output_image, rect_crop, cv::Scalar(255, 255, 255), 2);
    else
      cv::rectangle(output_image, rect_crop, cv::Scalar(255, 255, 255));

    // Project lights
    const auto& projection_roi = light->region.projection_roi;
    const cv::Rect projection_rect(projection_roi.x, projection_roi.y,
                                   projection_roi.width, projection_roi.height);
    cv::rectangle(output_image, projection_rect, cv::Scalar(255, 0, 0), 3);

    // Detect lights
    const auto& rectified_roi = light->region.detection_roi;
    const cv::Rect rectified_rect(rectified_roi.x, rectified_roi.y,
                                  rectified_roi.width, rectified_roi.height);
    cv::Scalar tl_color;
    std::map<base::TLColor, TLInfo>::iterator itor =
        s_tl_infos.find(light->status.color);
    if (itor != s_tl_infos.end()) {
      tl_color = itor->second.tl_color_;
      tl_string = itor->second.tl_string_;
    } else {
      tl_color = cv::Scalar(255, 255, 255);
      tl_string = "UNKNOWN";
    }
    snprintf(str, sizeof(str), "ID:%s C:%.3lf", light->id.c_str(),
             light->status.confidence);
    cv::rectangle(output_image, rectified_rect, tl_color, 2);
    cv::putText(output_image, str,
                cv::Point(rectified_roi.x + 30,
                          rectified_roi.y + rectified_roi.height + 30),
                cv::FONT_HERSHEY_DUPLEX, 1.0, tl_color, 2);
  }

  // Show text of voting results
  std::map<base::TLColor, TLInfo>::iterator itor =
      s_tl_infos.find(detected_trafficlight_color_);
  if (itor != s_tl_infos.end()) {
    tl_color = itor->second.tl_color_;
    tl_string = itor->second.tl_string_ex_;
  } else {
    tl_color = cv::Scalar(255, 255, 255);
    tl_string = "UNKNOWN traffic light";
  }
  double all = cnt_r_ + cnt_g_ + cnt_y_ + cnt_u_;
  if (all < 0.0001) {
    all = 1.0;
  }
  cv::putText(output_image, tl_string, cv::Point(10, 90),
              cv::FONT_HERSHEY_DUPLEX, 2.0, tl_color, 3);

  snprintf(str, sizeof(str), "Red lights:%.2f", cnt_r_ / all);
  cv::putText(output_image, str, cv::Point(10, 150), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 0, 255), 3);
  snprintf(str, sizeof(str), "Green lights:%.2f", cnt_g_ / all);
  cv::putText(output_image, str, cv::Point(10, 200), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 255, 0), 3);
  snprintf(str, sizeof(str), "Yellow lights:%.2f", cnt_y_ / all);
  cv::putText(output_image, str, cv::Point(10, 250), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 255, 255), 3);
  snprintf(str, sizeof(str), "Unknown lights:%.2f", cnt_u_ / all);
  cv::putText(output_image, str, cv::Point(10, 300), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(255, 255, 255), 3);

  std::string folder = "/apollo/data/debug_vis/";
  if (!apollo::cyber::common::EnsureDirectory(folder)) {
    AINFO << "EnsureDirectory folder " << folder << " error.";
  }
  cv::resize(output_image, output_image, cv::Size(), 0.5, 0.5);
  cv::imwrite(absl::StrCat(folder, std::to_string(frame.timestamp), ".jpg"),
      output_image);
  // todo(huqilin): need to create debug_vis folder.
  // cv::imshow("Apollo traffic light detection", output_image);
  cv::waitKey(1);
}

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
