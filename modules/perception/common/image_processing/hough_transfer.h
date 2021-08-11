/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <memory.h>
#include <cmath>

#include <set>
#include <string>
#include <vector>

namespace apollo {
namespace perception {
namespace common {

struct HoughLine {
  float r = 0.0f;
  float theta = 0.0f;
  float length = 0.0f;
  int vote_num = 0;
  std::vector<int> pts;
};

class HoughTransfer {
 public:
  HoughTransfer();
  ~HoughTransfer() = default;

  // step1
  // @brief: initiate
  // @params[IN] img_w, img_h: width and height of binary image
  //             d_r, d_theta: discretization step of r and theta
  //                           in polar coordinates
  bool Init(int img_w, int img_h, float d_r, float d_theta);

  // step2
  // @brief: HoughTransform in 2D binary image
  // @params[IN] image: 2D binary image.
  //             with_distribute: flag to control whether to calculate element
  //                              length,vote_num,pts in HoughLine
  bool ImageVote(const std::vector<int>& image, bool with_distribute);

  // @brief: transform one point to parameter space in polar coodinates and vote
  // @params[IN] x, y: pos in image.
  //             with_distribute: flag to control whether to calculate element
  //                              length,vote_num,pts in HoughLine
  void PointVote(int x, int y, bool with_distribute);
  // @params[IN] pos = y*img_w + x
  void PointVote(int pos, bool with_distribute);

  // step3
  // @brief get lines
  // @params[IN] min_pt_num: minimum points on the same line.
  //             r_neibor, theta_neibor: query region
  //             with_distribute: flag to control whether to calculate element
  //                              length,vote_num,pts in HoughLine
  //             lines: save lines detected.
  bool GetLines(int min_pt_num, int r_neibor, int theta_neibor,
                bool with_distribute, std::vector<HoughLine>* lines) const;

  unsigned int MemoryConsume() const;

  // prepared state not change.
  // when we use hough with with_distribute mode in large image long time,
  // memory consume maybe too large, so use this func to free no used cache.
  void FreeCache();

  void ResetMaps(bool with_distribute);

  inline std::string name() const { return "HoughTransfer"; }

  inline bool is_prepared() const { return prepared_; }

  inline const std::vector<int>& get_vote_map() const { return vote_map_; }
  inline const std::vector<std::vector<int>>& get_distribute_map() const {
    return distribute_map_;
  }

  inline float get_d_r() const { return d_r_; }
  inline float get_d_theta() const { return d_theta_; }
  inline int get_img_w() const { return img_w_; }
  inline int get_img_h() const { return img_h_; }
  inline int get_r_size() const { return r_size_; }
  inline int get_theta_size() const { return theta_size_; }

  void ClearWithShrink();

 private:
  bool CheckPrepared() const;

  void GetMaxVotes(int min_pt_num, int r_neibor, int theta_neibor, int r_step,
                   int theta_step, std::set<int>* max_vote_lines) const;

  bool VotePosToHoughLine(int vote_pos, bool with_distribute,
                          HoughLine* out_line) const;

  bool prepared_;
  float d_r_;
  float d_theta_;
  int img_w_;
  int img_h_;
  int r_size_;
  int theta_size_;
  int vote_reserve_size_;
  std::vector<int> vote_map_;
  std::vector<std::vector<int>> query_map_;
  std::vector<std::vector<int>> distribute_map_;
};

}  // namespace common
}  // namespace perception
}  // namespace apollo
