/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "smartereye_component.h"

#include "smartereye_handler.h"

namespace apollo {
namespace drivers {
namespace smartereye {

SmartereyeComponent::~SmartereyeComponent() {
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

bool SmartereyeComponent::Init() {
  camera_config_ = std::make_shared<Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_config_.get())) {
    return false;
  }
  AINFO << "SmartereyeDevice config: " << camera_config_->DebugString();

  camera_device_.reset(new SmartereyeDevice());
  camera_device_->init(camera_config_);

  device_wait_ = camera_config_->device_wait_ms();
  spin_rate_ = static_cast<uint32_t>((1.0 / camera_config_->spin_rate()) * 1e6);

  b_ispolling_ = false;

  SetCallback();

  writer_ = node_->CreateWriter<Image>(camera_config_->channel_name_image());
  SmartereyeObstacles_writer_ =
      node_->CreateWriter<SmartereyeObstacles>("/apollo/sensor/smartereye/smartereyeobstacles");
  SmartereyeLanemark_writer_ = node_->CreateWriter<SmartereyeLanemark>(
      "/apollo/sensor/smartereye/smartereyelanemark");

  async_result_ = cyber::Async(&SmartereyeComponent::run, this);

  return true;
}

void SmartereyeComponent::run() {
  running_.exchange(true);
  while (!cyber::IsShutdown()) {
    camera_device_->poll();
    cyber::SleepFor(std::chrono::microseconds(spin_rate_));
  }
}

bool SmartereyeComponent::SetCallback() {
  CallbackFunc fun = std::bind(&SmartereyeComponent::Callback, this, _1);
  camera_device_->SetCallback(fun);

  return true;
}

bool SmartereyeComponent::Callback(RawImageFrame *rawFrame) {
  if (rawFrame->frameId == FrameId::Compound ||
      rawFrame->frameId == FrameId::LaneExt) {
    processFrame(rawFrame->frameId, (char *)rawFrame + sizeof(RawImageFrame),
                 (char *)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame),
                 rawFrame->time, rawFrame->width, rawFrame->height);
  } else {
    processFrame(rawFrame->frameId, (char *)rawFrame + sizeof(RawImageFrame),
                 rawFrame->dataSize, rawFrame->width, rawFrame->height,
                 rawFrame->format);
  }

  return true;
}

void SmartereyeComponent::processFrame(int frameId, char *image, char *extended,
                                       int64_t time, int width, int height) {
  switch (frameId) {
    case FrameId::Compound: {
      FrameDataExtHead *header = reinterpret_cast<FrameDataExtHead *>(extended);
      int blockNum = ((int *)header->data)[0];
      OutputObstacles *obsDataPack =
          (OutputObstacles *)(((int *)header->data) + 2);

      SmartereyeObstacles pbSmartereyeObstacles;
      pbSmartereyeObstacles.mutable_header()->set_frame_id(
          camera_config_->frame_id());
      pbSmartereyeObstacles.set_num_obstacles(blockNum);

      OutputObstacle pbOutputObstacle[blockNum];
      for (int j = 0; j < blockNum; j++) {
        pbOutputObstacle[j].set_currentspeed(obsDataPack[j].currentSpeed);
        pbOutputObstacle[j].set_framerate(obsDataPack[j].frameRate);
        pbOutputObstacle[j].set_trackid(obsDataPack[j].trackId);
        pbOutputObstacle[j].set_trackframenum(obsDataPack[j].trackFrameNum);
        pbOutputObstacle[j].set_statelabel(obsDataPack[j].stateLabel);
        pbOutputObstacle[j].set_classlabel(obsDataPack[j].classLabel);
        pbOutputObstacle[j].set_continuouslabel(obsDataPack[j].continuousLabel);
        pbOutputObstacle[j].set_fuzzyestimationvalid(
            obsDataPack[j].fuzzyEstimationValid);
        pbOutputObstacle[j].set_obstacletype(
            (OutputObstacle_RecognitionType)obsDataPack[j].obstacleType);
        pbOutputObstacle[j].set_avgdisp(obsDataPack[j].avgDistanceZ);
        pbOutputObstacle[j].set_avgdistancez(obsDataPack[j].avgDistanceZ);
        pbOutputObstacle[j].set_neardistancez(obsDataPack[j].nearDistanceZ);
        pbOutputObstacle[j].set_fardistancez(obsDataPack[j].farDistanceZ);
        pbOutputObstacle[j].set_real3dleftx(obsDataPack[j].real3DLeftX);
        pbOutputObstacle[j].set_real3drightx(obsDataPack[j].real3DRightX);
        pbOutputObstacle[j].set_real3dcenterx(obsDataPack[j].real3DCenterX);
        pbOutputObstacle[j].set_real3dupy(obsDataPack[j].real3DUpY);
        pbOutputObstacle[j].set_real3dlowy(obsDataPack[j].real3DLowY);
        pbOutputObstacle[j].set_firstpointx(obsDataPack[j].firstPointX);
        pbOutputObstacle[j].set_firstpointy(obsDataPack[j].firstPointY);
        pbOutputObstacle[j].set_secondpointx(obsDataPack[j].secondPointX);
        pbOutputObstacle[j].set_secondpointy(obsDataPack[j].secondPointY);
        pbOutputObstacle[j].set_thirdpointx(obsDataPack[j].thirdPointX);
        pbOutputObstacle[j].set_thirdpointy(obsDataPack[j].thirdPointY);
        pbOutputObstacle[j].set_fourthpointx(obsDataPack[j].fourthPointX);
        pbOutputObstacle[j].set_fourthpointy(obsDataPack[j].fourthPointY);
        pbOutputObstacle[j].set_fuzzyrelativedistancez(
            obsDataPack[j].fuzzyRelativeDistanceZ);
        pbOutputObstacle[j].set_fuzzyrelativespeedz(
            obsDataPack[j].fuzzyRelativeSpeedZ);
        pbOutputObstacle[j].set_fuzzycollisiontimez(
            obsDataPack[j].fuzzyCollisionTimeZ);
        pbOutputObstacle[j].set_fuzzycollisionx(obsDataPack[j].fuzzyCollisionX);
        pbOutputObstacle[j].set_fuzzy3dwidth(obsDataPack[j].fuzzy3DWidth);
        pbOutputObstacle[j].set_fuzzy3dcenterx(obsDataPack[j].fuzzy3DCenterX);
        pbOutputObstacle[j].set_fuzzy3dleftx(obsDataPack[j].fuzzy3DLeftX);
        pbOutputObstacle[j].set_fuzzy3drightx(obsDataPack[j].fuzzy3DRightX);
        pbOutputObstacle[j].set_fuzzy3dheight(obsDataPack[j].fuzzy3DHeight);
        pbOutputObstacle[j].set_fuzzy3dupy(obsDataPack[j].fuzzy3DUpY);
        pbOutputObstacle[j].set_fuzzy3dlowy(obsDataPack[j].fuzzy3DLowY);
        pbOutputObstacle[j].set_fuzzyrelativespeedcenterx(
            obsDataPack[j].fuzzyRelativeSpeedCenterX);
        pbOutputObstacle[j].set_fuzzyrelativespeedleftx(
            obsDataPack[j].fuzzyRelativeSpeedLeftX);
        pbOutputObstacle[j].set_fuzzyrelativespeedrightx(
            obsDataPack[j].fuzzyRelativeSpeedRightX);
        (*pbSmartereyeObstacles.mutable_output_obstacles())[j] =
            pbOutputObstacle[j];
        SmartereyeObstacles_writer_->Write(pbSmartereyeObstacles);
      }

      static unsigned char *rgbBuf = new unsigned char[width * height * 3];
      RoadwayPainter::imageGrayToRGB((unsigned char *)image, rgbBuf, width,
                                     height);
      ObstaclePainter::paintObstacle(header->data, rgbBuf, width, height, true,
                                     false);

      Image pb_image;
      pb_image.mutable_header()->set_frame_id(camera_config_->frame_id());
      pb_image.set_width(width);
      pb_image.set_height(height);
      pb_image.set_data(rgbBuf, width * height * 3);
      pb_image.set_encoding("rgb8");
      pb_image.set_step(width * 3);
      pb_image.mutable_header()->set_timestamp_sec(
          cyber::Time::Now().ToSecond());
      pb_image.set_measurement_time(time);

      writer_->Write(pb_image);
    } break;
    case FrameId::LaneExt: {
      FrameDataExtHead *header = reinterpret_cast<FrameDataExtHead *>(extended);
      LdwDataPack *ldwDataPack = (LdwDataPack *)header->data;
      AINFO << "ldw degree is: "
            << ldwDataPack->roadway.left_Lane.left_Boundary.degree;
      SmartereyeLanemark pbSmartereyeLanemark;
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_roadway()
          ->set_width_0(ldwDataPack->roadway.width[0]);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_roadway()
          ->set_width_1(ldwDataPack->roadway.width[1]);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_roadway()
          ->set_width_2(ldwDataPack->roadway.width[2]);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_roadway()
          ->set_is_tracking(ldwDataPack->roadway.isTracking);
      pbSmartereyeLanemark.mutable_lane_road_data()->set_softstatus(
          (LdwSoftStatus)ldwDataPack->softStatus);
      pbSmartereyeLanemark.mutable_lane_road_data()->set_steerstatus(
          (LdwSteerStatus)ldwDataPack->steerStatus);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_lens()
          ->set_x_image_focal(ldwDataPack->lens.xImageFocal);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_lens()
          ->set_y_image_focal(ldwDataPack->lens.yImageFocal);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_lens()
          ->set_xratio_focal_pixel(ldwDataPack->lens.xRatioFocalToPixel);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_lens()
          ->set_yratio_focal_pixel(ldwDataPack->lens.yRatioFocalToPixel);
      pbSmartereyeLanemark.mutable_lane_road_data()
          ->mutable_lens()
          ->set_mountingheight(ldwDataPack->lens.mountingHeight);
      pbSmartereyeLanemark.mutable_lane_road_data()->mutable_lens()->set_mcosrx(
          ldwDataPack->lens.mCosRx);
      pbSmartereyeLanemark.mutable_lane_road_data()->mutable_lens()->set_msinrx(
          ldwDataPack->lens.mSinRx);
      pbSmartereyeLanemark.mutable_lane_road_data()->mutable_lens()->set_mcosry(
          ldwDataPack->lens.mCosRy);
      pbSmartereyeLanemark.mutable_lane_road_data()->mutable_lens()->set_msinry(
          ldwDataPack->lens.mSinRy);
      SmartereyeLanemark_writer_->Write(pbSmartereyeLanemark);

      static unsigned char *rgbBuf = new unsigned char[width * height * 3];
      RoadwayPainter::imageGrayToRGB((unsigned char *)image, rgbBuf, width,
                                     height);
      bool mIsLaneDetected =
          RoadwayPainter::paintRoadway(header->data, rgbBuf, width, height);
      AINFO << mIsLaneDetected;

      Image pb_image;
      pb_image.mutable_header()->set_frame_id(camera_config_->frame_id());
      pb_image.set_width(width);
      pb_image.set_height(height);
      pb_image.set_data(rgbBuf, width * height * 3);
      pb_image.set_encoding("rgb8");
      pb_image.set_step(width * 3);
      pb_image.mutable_header()->set_timestamp_sec(
          cyber::Time::Now().ToSecond());
      pb_image.set_measurement_time(time);

      writer_->Write(pb_image);
    }
    default:
      break;
  }
}

void SmartereyeComponent::processFrame(int frameId, char *image,
                                       uint32_t dataSize, int width, int height,
                                       int frameFormat) {
  switch (frameId) {
    case FrameId::Lane: {
      AINFO << "case FrameId::Lane:";
      LdwDataPack *ldwDataPack = (LdwDataPack *)image;
      int degree = ldwDataPack->roadway.left_Lane.left_Boundary.degree;
      AINFO << "ldw degree is: " << degree;
    } break;
    case FrameId::Obstacle: {
      AINFO << "case FrameId::Obstacle:";
      int blockNum = ((int *)image)[0];
      AINFO << "blockNum is: " << blockNum;
    } break;
    case FrameId::Disparity: {
      AINFO << "case FrameId::Disparity:";
    } break;
    case FrameId::CalibLeftCamera: {
      AINFO << "case FrameId::CalibLeftCamera:";
    } break;
    default:
      break;
  }
}

}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
