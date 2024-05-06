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
#include "modules/perception/common/algorithm/image_processing/hough_transfer.h"

namespace apollo {
namespace perception {
namespace algorithm {

HoughTransfer::HoughTransfer()
    : prepared_(false),
      d_r_(0.0),
      d_theta_(0.0),
      img_w_(0),
      img_h_(0),
      r_size_(0),
      theta_size_(0),
      vote_reserve_size_(10),
      vote_map_(),
      query_map_(),
      distribute_map_() {}

// step1
// @brief: initiate
// @params[IN] img_w, img_h: width and height of binary image
//             d_r, d_theta: discretization step of r and theta
//                           in polar coordinates
bool HoughTransfer::Init(int img_w, int img_h, float d_r, float d_theta) {
  img_w_ = img_w;
  img_h_ = img_h;
  d_r_ = d_r;
  d_theta_ = static_cast<float>(M_PI) * d_theta / 180.0f;
  r_size_ = static_cast<int>(
      2 * sqrtf(static_cast<float>(img_w_ * img_w_ + img_h_ * img_h_)) / d_r_);
  theta_size_ = static_cast<int>(M_PI / d_theta_);

  ClearWithShrink();
  // init vote map
  vote_map_.resize(r_size_ * theta_size_, 0);
  // init query map
  query_map_.resize(img_w_ * img_h_);
  for (auto& query : query_map_) {
    query.resize(theta_size_, 0);
  }

  for (int theta_idx = 0; theta_idx < theta_size_; ++theta_idx) {
    float cur_theta = d_theta_ * static_cast<float>(theta_idx);
    for (int img_pos = 0; img_pos < img_w_ * img_h_; ++img_pos) {
      int w = img_pos % img_w_;
      int h = img_pos / img_w_;
      int r =
          static_cast<int>((cos(cur_theta) * w + sin(cur_theta) * h) / d_r_) +
          r_size_ / 2;
      if (0 <= r && r < r_size_) {
        query_map_[img_pos][theta_idx] = r * theta_size_ + theta_idx;
      }
    }
  }
  // init distribute map
  distribute_map_.resize(r_size_ * theta_size_);
  for (auto& distribute : distribute_map_) {
    distribute.reserve(vote_reserve_size_);
  }

  if (CheckPrepared()) {
    prepared_ = true;
  } else {
    ClearWithShrink();
    prepared_ = false;
  }
  return prepared_;
}

// step2
// @brief: HoughTransform in 2D binary image
// @params[IN] image: 2D binary image.
//             with_distribute: flag to control whether to calculate element
//                              length,vote_num,pts in HoughLine
bool HoughTransfer::ImageVote(const std::vector<int>& image,
                              bool with_distribute) {
  if (image.size() != query_map_.size()) {
    return false;
  }
  ResetMaps(with_distribute);
  for (size_t i = 0; i < image.size(); ++i) {
    if (image[i] > 0) {
      int x = static_cast<int>(i) % img_w_;
      int y = static_cast<int>(i) / img_w_;
      PointVote(x, y, with_distribute);
    }
  }
  return true;
}

// @brief: transform one point to parameter space in polar coodinates and vote
// @params[IN] x, y: pos in image.
//             with_distribute: flag to control whether to calculate element
//                              length,vote_num,pts in HoughLine
void HoughTransfer::PointVote(int x, int y, bool with_distribute) {
  const int pos = y * img_w_ + x;
  PointVote(pos, with_distribute);
}

// @paramas[IN] pos: pos = y*img_w +x
void HoughTransfer::PointVote(int pos, bool with_distribute) {
  for (int theta_idx = 0; theta_idx < theta_size_; ++theta_idx) {
    ++vote_map_[query_map_[pos][theta_idx]];
    if (with_distribute) {
      distribute_map_[query_map_[pos][theta_idx]].push_back(pos);
    }
  }
}

// step3
// @brief get lines
// @params[IN] min_pt_num: minimum points on the same line.
//             r_neibor, theta_neibor: query region
//             with_distribute: flag to control whether to calculate element
//                              length,vote_num,pts in HoughLine
//             lines: save lines detected.
bool HoughTransfer::GetLines(int min_pt_num, int r_neibor, int theta_neibor,
                             bool with_distribute,
                             std::vector<HoughLine>* lines) const {
  if (!lines) {
    return false;
  }
  int r_step = 2 * r_neibor + 1;
  int theta_step = 2 * theta_neibor + 1;

  // search one vote neighbor for max_vote position in the region
  std::set<int> max_vote_lines;
  GetMaxVotes(min_pt_num, r_neibor, theta_neibor, r_step, theta_step,
              &max_vote_lines);

  // from max vote pos to get line params
  lines->resize(max_vote_lines.size());
  int idx = 0;
  for (auto i = max_vote_lines.begin(); i != max_vote_lines.end(); ++i) {
    if (!VotePosToHoughLine(*i, with_distribute, &(*lines)[idx++])) {
      return false;
    }
  }
  return true;
}

unsigned int HoughTransfer::MemoryConsume() const {
  unsigned int size = 0;
  if (is_prepared()) {
    size +=
        static_cast<unsigned int>(vote_map_.capacity() * sizeof(vote_map_[0]));
    size += static_cast<unsigned int>(query_map_.capacity() *
                                      sizeof(query_map_[0]));
    size += static_cast<unsigned int>(theta_size_ * query_map_.size() *
                                      sizeof(query_map_[0][0]));
    size += static_cast<unsigned int>(distribute_map_.capacity() *
                                      sizeof(distribute_map_[0]));
    for (const auto& distribute : distribute_map_) {
      size += static_cast<unsigned int>(distribute.capacity() *
                                        sizeof(distribute[0]));
    }
  }
  return size;
}

// prepared state not change.
// when we use hough with with_distribute mode in large image long time,
// memory consume maybe too large, so use this func to free no used cache.
void HoughTransfer::FreeCache() {
  for (auto& distribute : distribute_map_) {
    distribute.shrink_to_fit();
  }
}

void HoughTransfer::ResetMaps(bool with_distribute) {
  memset(vote_map_.data(), 0, vote_map_.size() * sizeof(vote_map_[0]));
  if (with_distribute) {
    for (auto& distribute : distribute_map_) {
      distribute.clear();
    }
  }
}

void HoughTransfer::ClearWithShrink() {
  vote_map_.clear();
  vote_map_.shrink_to_fit();
  query_map_.clear();
  query_map_.shrink_to_fit();
  distribute_map_.clear();
  distribute_map_.shrink_to_fit();
  prepared_ = false;
}

bool HoughTransfer::CheckPrepared() const {
  if (static_cast<int>(vote_map_.size()) != r_size_ * theta_size_) {
    return false;
  }
  if (static_cast<int>(query_map_.size()) != img_w_ * img_h_) {
    return false;
  }
  if (static_cast<int>(distribute_map_.size()) != r_size_ * theta_size_) {
    return false;
  }
  if (vote_map_.empty() || query_map_.empty() || distribute_map_.empty()) {
    return false;
  }
  return true;
}

void HoughTransfer::GetMaxVotes(int min_pt_num, int r_neibor, int theta_neibor,
                                int r_step, int theta_step,
                                std::set<int>* max_vote_lines) const {
  for (int i = r_neibor; i < r_size_ - r_neibor; i += r_step) {
    for (int j = theta_neibor; j < theta_size_ - theta_neibor;
         j += theta_step) {
      int max_pos = -1;
      int max_vote = min_pt_num;
      for (int m = -r_neibor; m <= r_neibor; ++m) {
        for (int n = -theta_neibor; n <= theta_neibor; ++n) {
          int neibor_pos = (i + m) * theta_size_ + j + n;
          if (vote_map_[neibor_pos] >= max_vote) {
            max_pos = neibor_pos;
            max_vote = vote_map_[neibor_pos];
          }
        }
      }
      if (max_pos >= 0) {
        max_vote_lines->insert(max_pos);
      }
    }
  }
}

bool HoughTransfer::VotePosToHoughLine(int vote_pos, bool with_distribute,
                                       HoughLine* out_line) const {
  if (!out_line) {
    return false;
  }
  out_line->r = static_cast<float>(vote_pos / theta_size_ - r_size_ / 2) * d_r_;
  out_line->theta = static_cast<float>(vote_pos % theta_size_) * d_theta_;
  out_line->vote_num = vote_map_[vote_pos];
  if (with_distribute) {
    if (out_line->vote_num !=
        static_cast<int>(distribute_map_[vote_pos].size())) {
      return false;
    }
    out_line->pts = distribute_map_[vote_pos];
    const int start_pos = distribute_map_[vote_pos][0];
    const int end_pos = distribute_map_[vote_pos][out_line->vote_num - 1];
    const int start_x = start_pos % img_w_;
    const int start_y = start_pos / img_w_;
    const int end_x = end_pos % img_w_;
    const int end_y = end_pos / img_w_;
    out_line->length =
        sqrtf(static_cast<float>((start_x - end_x) * (start_x - end_x) +
                                 (start_y - end_y) * (start_y - end_y)));
  }
  return true;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
