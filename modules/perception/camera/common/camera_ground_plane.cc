/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/common/camera_ground_plane.h"

#include <utility>

#include "cyber/common/log.h"
#include "modules/perception/common/i_lib/da/i_ransac.h"
#include "modules/perception/common/i_lib/geometry/i_util.h"

namespace apollo {
namespace perception {
namespace camera {

void ConvertGround3ToGround4(const float &baseline,
                             const std::vector<float> &k_mat,
                             const std::vector<float> &ground3,
                             std::vector<float> *ground4) {
  CHECK_GT(baseline, 0.0f);
  CHECK_EQ(k_mat.size(), 9);
  CHECK_EQ(ground3.size(), 3);
  CHECK_NOTNULL(ground4);
  CHECK_EQ(ground4->size(), 4);
  const float &b = baseline;
  const float &fx = k_mat[0];
  const float &fy = k_mat[4];
  const float &cy = k_mat[5];
  ground4->data()[0] = 0.0f;
  ground4->data()[1] = ground3[0] * fy;
  ground4->data()[2] = ground3[0] * cy + ground3[2];
  ground4->data()[3] = ground3[1] * b * fx;
  float norm = common::ISqrt(common::ISquaresum3(ground4->data()));
  common::IScale4(ground4->data(), common::IRec(norm));
}

bool ConvertGround4ToGround3(const float &baseline,
                             const std::vector<float> &k_mat,
                             const std::vector<float> &ground4,
                             std::vector<float> *ground3) {
  CHECK_GT(baseline, 0.0f);
  CHECK_EQ(k_mat.size(), 9);
  CHECK_EQ(ground4.size(), 4);
  CHECK_NOTNULL(ground3);
  CHECK_EQ(ground3->size(), 3);
  // normalization
  float p[4] = {ground4[0], ground4[1], ground4[2], ground4[3]};
  float norm = common::ISqrt(common::ISquaresum3(p));
  common::IScale4(p, common::IRec(norm));
  if (p[0] > 1e-3) {
    AERROR << "Have roll in the ground plane: " << p[0];
    ground3->assign(3, 0.f);
    return false;
  }
  // no roll
  const float &b = baseline;
  const float &fx = k_mat[0];
  const float &fy = k_mat[4];
  const float &cy = k_mat[5];
  ground3->data()[0] = p[1] * common::IRec(fy);
  ground3->data()[1] = p[3] * common::IRec(b * fx);
  ground3->data()[2] = p[2] - ground3->data()[0] * cy;
  return true;
}

void GetGroundPlanePitchHeight(const float &baseline,
                               const std::vector<float> &k_mat,
                               const std::vector<float> &ground3, float *pitch,
                               float *cam_height) {
  CHECK_GT(baseline, 0.0f);
  CHECK_EQ(k_mat.size(), 9);
  CHECK_EQ(ground3.size(), 3);
  CHECK_NOTNULL(pitch);
  CHECK_NOTNULL(cam_height);
  std::vector<float> ground4(4, 0.0f);
  ConvertGround3ToGround4(baseline, k_mat, ground3, &ground4);
  if (ground4[3] > 0.0f) {
    common::IScale4(ground4.data(), -1.0f);
  }
  *cam_height = -ground4[3];
  double cos_pitch = ground4[1];
  double sin_pitch = -ground4[2];
  *pitch = static_cast<float>(atan2(sin_pitch, cos_pitch));
}

void GetGround3FromPitchHeight(const std::vector<float> &k_mat,
                               const float &baseline, const float &pitch,
                               const float &cam_height,
                               std::vector<float> *ground3) {
  CHECK_EQ(k_mat.size(), 9);
  CHECK_GT(baseline, 0.0f);
  CHECK_GT(cam_height, 0.0f);
  CHECK_NOTNULL(ground3);
  CHECK_EQ(ground3->size(), 3);
  float sin_pitch = static_cast<float>(sin(pitch));
  float cos_pitch = static_cast<float>(cos(pitch));
  std::vector<float> ground4 = {0, cos_pitch, -sin_pitch, -cam_height};
  ConvertGround4ToGround3(baseline, k_mat, ground4, ground3);
}

GroundPlaneTracker::GroundPlaneTracker(int track_length) {
  if (track_length <= 0) {
    AERROR << "track_length, " << track_length << ", should be positive";
  }
  pitch_height_inlier_tracks_.resize(track_length * 3);
  const_weight_temporal_.resize(track_length, 0.0f);

  weight_.resize(track_length, 0.0f);
  for (int i = track_length - 1; i >= 0; --i) {
    const_weight_temporal_.at(i) =
        common::IPow(common::ISqrt(2.0f), track_length - 1 - i);
  }

  // normalize
  float accm_sum = common::ISum(const_weight_temporal_.data(), track_length);
  common::IScale(const_weight_temporal_.data(), track_length,
                 common::IRec(accm_sum));

  head_ = track_length;
}

void GroundPlaneTracker::Push(const std::vector<float> &ph,
                              const float &inlier_ratio) {
  CHECK_EQ(ph.size(), 2);
  int i = 0;
  int length = static_cast<int>(pitch_height_inlier_tracks_.size());
  if (head_ == 0) {
    if (length > 3) {                      // move backwards
      for (i = length - 1; i >= 3; i--) {  // 3: pitch, height, inlier-number
        pitch_height_inlier_tracks_[i] = pitch_height_inlier_tracks_[i - 3];
      }
    }
  } else {  // _head >= zero
    head_ -= 1;
  }

  // fill the head_ (fresh input)
  int head3 = head_ * 3;
  pitch_height_inlier_tracks_[head3] = ph[0];
  pitch_height_inlier_tracks_[head3 + 1] = ph[1];
  pitch_height_inlier_tracks_[head3 + 2] = inlier_ratio;
}

void GroundPlaneTracker::GetGround(float *pitch, float *cam_height) {
  CHECK_NOTNULL(pitch);
  CHECK_NOTNULL(cam_height);
  int i = 0;
  int length = static_cast<int>(pitch_height_inlier_tracks_.size() / 3);
  if (!length) {
    *pitch = *cam_height = 0.0f;
    return;
  } else if (length == 1) {
    *pitch = pitch_height_inlier_tracks_[0];
    *cam_height = pitch_height_inlier_tracks_[1];
    return;
  }

  float ph[2] = {0};
  float w = 0.0f;
  for (i = head_; i < length; i++) {
    w = pitch_height_inlier_tracks_.at(i * 3 + 2);
    weight_.at(i) = const_weight_temporal_.at(i) * w;
  }

  // normalize factor
  float accm_wei = common::ISum(weight_.data() + head_, (length - head_));

  ph[0] = ph[1] = 0.0f;
  for (i = head_; i < length; i++) {
    w = weight_.at(i);
    int i3 = i * 3;
    ph[0] += pitch_height_inlier_tracks_.at(i3) * w;
    ph[1] += pitch_height_inlier_tracks_.at(i3 + 1) * w;
  }
  ph[0] = common::IDiv(ph[0], accm_wei);
  ph[1] = common::IDiv(ph[1], accm_wei);
  *pitch = ph[0];
  *cam_height = ph[1];
}

void GroundPlaneTracker::Restart() {
  unsigned int track_length =
      static_cast<unsigned int>(pitch_height_inlier_tracks_.size() / 3);
  auto &data = pitch_height_inlier_tracks_;
  unsigned int i = 0;
  for (; i < track_length; ++i) {
    int i3 = i * 3;
    data.at(i3) = data.at(i3 + 1) = data.at(i3 + 2) = 0.0f;
    weight_.at(i) = 0.0f;
  }
  head_ = track_length;  // reset to init value
}

void CameraGroundPlaneParams::SetDefault() {
  min_nr_samples = 6;   // 40
  nr_frames_track = 3;  // 2
  max_tilt_angle = common::IDegreeToRadians(10.0f);
  max_camera_ground_height = 2.5f;
  min_inlier_ratio = 0.5f;
  /*
  thres_inlier_plane_fitting = 1.5f;  // in pixel
  */
  thres_inlier_plane_fitting = 0.0035f;  // in reversed depth, 3m@30m
}

bool CameraGroundPlaneDetector::DetetGround(float pitch, float camera_height,
                                            float *vd, int count_vd,
                                            const std::vector<float> &plane) {
  ground_is_valid_ = false;

  std::vector<float> ground3(l_, l_ + 3);
  std::vector<float> k_mat(k_mat_, k_mat_ + 9);

  if (!plane.empty()) {  // assigned from outside
    //    GetGeneralGroundPlaneModelReversed(k_mat_, baseline_, plane.data(),
    //    l_);
    ConvertGround4ToGround3(baseline_, k_mat, plane, &ground3);
    FillGroundModel(ground3);
    AINFO << "set ground plane from outside: " << plane[0] << ", " << plane[1]
          << ", " << plane[2] << ", " << plane[3];
    ground_is_valid_ = true;
    return true;
  } else {
    bool success = false;
    float inlier_ratio = 0.0f;
    std::vector<float> ph(2, 0);
    if (CameraGroundPlaneDetector::DetectGroundFromSamples(vd, count_vd,
                                                           &inlier_ratio)) {
      ADEBUG << "l: " << l_[0] << ", " << l_[1] << ", " << l_[2];
      ground3.assign(l_, l_ + 3);
      GetGroundPlanePitchHeight(baseline_, k_mat, ground3, &ph[0], &ph[1]);
      ADEBUG << "ph: " << ph[0] << ", " << ph[1];
      success = fabs(ph[0]) < params_.max_tilt_angle &&
                ph[1] < params_.max_camera_ground_height;
      if (success) {
        ground_plane_tracker_->Push(ph, inlier_ratio);
        ground_plane_tracker_->GetGround(&ph[0], &ph[1]);
        GetGround3FromPitchHeight(k_mat, baseline_, ph[0], ph[1], &ground3);
        FillGroundModel(ground3);
        ADEBUG << "l tracked: " << l_[0] << ", " << l_[1] << ", " << l_[2];
        ADEBUG << "ph tracked: " << ph[0] << ", " << ph[1];
      }
    }

    if (success) {
      ADEBUG << "succeed with inlier ratio: " << inlier_ratio;
      ground_is_valid_ = true;
      return true;
    }

    // backup using last successful frame or given pitch & height
    if (ground_plane_tracker_->GetCurTrackLength() > 0) {
      ground_plane_tracker_->GetGround(&ph[0], &ph[1]);
      GetGround3FromPitchHeight(k_mat, baseline_, ph[0], ph[1], &ground3);
      FillGroundModel(ground3);
    } else {
      CHECK(fabs(pitch) < params_.max_tilt_angle);
      CHECK(camera_height < params_.max_camera_ground_height);
      CHECK_GT(camera_height, 0.f);
      GetGround3FromPitchHeight(k_mat, baseline_, pitch, camera_height,
                                &ground3);
      FillGroundModel(ground3);
    }
    ground_plane_tracker_->Restart();
    return false;
  }
}

bool CameraGroundPlaneDetector::DetectGroundFromSamples(float *vd, int count_vd,
                                                        float *inlier_ratio) {
  if (vd == nullptr) {
    AERROR << "vd is nullptr";
    return false;
  }
  if (inlier_ratio == nullptr) {
    AERROR << "inlier_ratio is nullptr";
    return false;
  }
  *inlier_ratio = 0.0f;
  if (count_vd < params_.min_nr_samples) {
    l_[0] = l_[1] = l_[2] = 0.0f;
    return false;
  }

  double kMinInlierRatio = params_.min_inlier_ratio;
  float kThresInlier = params_.thres_inlier_plane_fitting;
  /*standard RANSAC solution*/
  float *vs = ss_flt_.data();
  float *ds = vs + count_vd;
  for (int i = 0; i < count_vd; ++i) {
    int i2 = i << 1;
    vs[i] = vd[i2];
    ds[i] = vd[i2 + 1];
  }
  int nr_inliers = 0;
  int *inliers = ss_int_.data();
  memset(inliers, 0, sizeof(int) * count_vd * 2);
  float p[2] = {0};

  if (!common::RobustBinaryFitRansac<float, 1, 1, 2, 2,
                                     GroundHypoGenFunc<float>,
                                     GroundFittingCostFunc<float>, nullptr>(
          vs, ds, count_vd, p, &nr_inliers, inliers, kThresInlier, false, true,
          0.99f, kMinInlierRatio)) {
    memset(l_, 0, sizeof(float) * 3);
    return false;
  } else {
    *inlier_ratio = static_cast<float>(nr_inliers) *
                    common::IRec(static_cast<float>(count_vd));
  }

  if (*inlier_ratio < kMinInlierRatio) {
    *inlier_ratio = 0.0f;
    memset(l_, 0, sizeof(float) * 3);
    return false;
  }

  // re-fit using inliers
  int count = 0;
  for (int i = 0; i < nr_inliers; ++i) {
    int i2 = inliers[i] << 1;
    int count2 = count << 1;
    std::swap(vd[i2], vd[count2]);
    std::swap(vd[i2 + 1], vd[count2 + 1]);
    ++count;
  }
  float l_best[3] = {0};
  common::ILineFit2dTotalLeastSquare(vd, l_best, count);
  memcpy(l_, l_best, sizeof(float) * 3);
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
