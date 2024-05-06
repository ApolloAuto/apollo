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
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_label_image.h"

#include <algorithm>
#include <utility>

#include "modules/perception/common/lidar/common/lidar_log.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_pool_types.h"

namespace apollo {
namespace perception {
namespace lidar {

void SppLabelImage::Init(size_t width, size_t height,
                         const std::string& sensor_name) {
  // simply release the last memory and allocate new one
  if (labels_) {
    algorithm::IFree2(&labels_);
  }
  width_ = width;
  height_ = height;
  sensor_name_ = sensor_name;
  labels_ = algorithm::IAlloc2<uint16_t>(static_cast<int>(height_),
                                      static_cast<int>(width_));
  memset(labels_[0], 0, sizeof(uint16_t) * width_ * height_);
  clusters_.clear();
}

void SppLabelImage::InitRangeMask(float range, float boundary_distance) {
  if (range_mask_) {
    algorithm::IFree2(&range_mask_);
  }
  range_mask_ = algorithm::IAlloc2<char>(static_cast<int>(height_),
                                      static_cast<int>(width_));
  memset(range_mask_[0], 0, sizeof(char) * width_ * height_);
  float meter_per_pixel = range * 2.0f / static_cast<float>(width_);
  size_t half_width = width_ / 2;
  size_t half_height = height_ / 2;
  for (size_t r = 0; r < height_; ++r) {
    for (size_t c = 0; c < width_; ++c) {
      float distance = sqrtf(
          powf((static_cast<float>(r) - static_cast<float>(half_height)), 2.f) +
          powf((static_cast<float>(c) - static_cast<float>(half_width)), 2.f));
      distance *= meter_per_pixel;
      if (distance <= boundary_distance) {
        range_mask_[r][c] = 1;
      }
    }
  }
}

void SppLabelImage::CollectClusterFromSppLabelImage() {
  size_t size = width_ * height_;
  // find max label
  uint16_t max_label = *(std::max_element(labels_[0], labels_[0] + size));
  clusters_.clear();
  SppClusterPool::Instance(sensor_name_).BatchGet(max_label, &clusters_);
  for (size_t y = 0; y < height_; ++y) {
    for (size_t x = 0; x < width_; ++x) {
      uint16_t& label = labels_[y][x];
      // label 0 is invalid, will be ignored
      if (label) {
        clusters_[label - 1]->pixels.push_back(
            static_cast<unsigned int>(y * width_ + x));
      }
    }
  }
}

void SppLabelImage::ProjectClusterToSppLabelImage() {
  memset(labels_[0], 0, sizeof(uint16_t) * width_ * height_);
  for (size_t n = 0; n < clusters_.size(); ++n) {
    auto& cluster = clusters_[n];
    for (auto& pixel : cluster->pixels) {
      labels_[0][pixel] = static_cast<uint16_t>(n + 1);
    }
  }
}

void SppLabelImage::FilterClusters(const float* confidence_map,
                                   float threshold) {
  for (auto& cluster : clusters_) {
    float sum = 0.f;
    for (auto& pixel : cluster->pixels) {
      sum += confidence_map[pixel];
    }
    sum = cluster->pixels.size() > 0
              ? sum / static_cast<float>(cluster->pixels.size())
              : sum;
    cluster->confidence = sum;
  }
  size_t current = 0;
  for (size_t n = 0; n < clusters_.size(); ++n) {
    if (clusters_[n]->confidence >= threshold) {
      if (current != n) {
        clusters_[current] = clusters_[n];
        for (auto& pixel : clusters_[current]->pixels) {
          labels_[0][pixel] = static_cast<uint16_t>(current + 1);
        }
      }
      ++current;
    } else {
      for (auto& pixel : clusters_[n]->pixels) {
        labels_[0][pixel] = 0;
      }
    }
  }
  clusters_.resize(current);
}

void SppLabelImage::FilterClusters(const float* confidence_map,
                                   const float* category_map,
                                   float confidence_threshold,
                                   float category_threshold) {
  std::vector<bool> is_valid;
  is_valid.reserve(clusters_.size());
  for (auto& cluster : clusters_) {
    char mask = 1;
    for (auto& pixel : cluster->pixels) {
      mask &= range_mask_[0][pixel];
    }
    float sum_confidence = 0.f;
    for (auto& pixel : cluster->pixels) {
      sum_confidence += confidence_map[pixel];
    }
    sum_confidence =
        cluster->pixels.size() > 0
            ? sum_confidence / static_cast<float>(cluster->pixels.size())
            : sum_confidence;
    cluster->confidence = sum_confidence;
    if (mask) {  // in range, use confidence estimation
      is_valid.push_back(cluster->confidence >= confidence_threshold);
    } else {  // out of range, use category estimation
      float sum_category = 0.f;
      for (auto& pixel : cluster->pixels) {
        sum_category += category_map[pixel];
      }
      sum_category =
          cluster->pixels.size() > 0
              ? sum_category / static_cast<float>(cluster->pixels.size())
              : sum_category;
      is_valid.push_back(sum_category >= category_threshold);
      // category is not stable, here we hack the confidence
      cluster->confidence =
          std::max(sum_confidence, confidence_threshold + 0.01f);
    }
  }
  size_t current = 0;
  for (size_t n = 0; n < clusters_.size(); ++n) {
    if (is_valid[n]) {
      if (current != n) {
        clusters_[current] = clusters_[n];
        for (auto& pixel : clusters_[current]->pixels) {
          labels_[0][pixel] = static_cast<uint16_t>(current + 1);
        }
      }
      ++current;
    } else {
      for (auto& pixel : clusters_[n]->pixels) {
        labels_[0][pixel] = 0;
      }
    }
  }
  clusters_.resize(current);
}

void SppLabelImage::CalculateClusterClass(const float* class_map,
                                          size_t class_num) {
  for (auto& cluster : clusters_) {
    cluster->class_prob.assign(class_num, 0.f);
  }
  size_t size = width_ * height_;
  for (size_t c = 0; c < class_num; ++c) {
    const float* class_map_ptr = class_map + c * size;
    for (auto& cluster : clusters_) {
      auto& probs = cluster->class_prob;
      for (auto& pixel : cluster->pixels) {
        probs[c] += class_map_ptr[pixel];
      }
    }
  }
  for (auto& cluster : clusters_) {
    auto& probs = cluster->class_prob;
    float sum = std::accumulate(probs.begin(), probs.end(), 0.f);
    if (sum > 1e-9) {
      for (auto& value : probs) {
        value /= sum;
      }
    }
    cluster->type = static_cast<SppClassType>(std::distance(
        probs.begin(), std::max_element(probs.begin(), probs.end())));
  }
}

void SppLabelImage::CalculateClusterHeading(const float* heading_map) {
  const float* heading_map_x_ptr = heading_map;
  const float* heading_map_y_ptr = heading_map + width_ * height_;

  for (size_t n = 0; n < clusters_.size(); ++n) {
    float heading_x = 0.f, heading_y = 0.f;
    for (auto pixel : clusters_[n]->pixels) {
      heading_x += heading_map_x_ptr[pixel];
      heading_y += heading_map_y_ptr[pixel];
    }
    clusters_[n]->yaw = std::atan2(heading_y, heading_x) * 0.5f;
  }
}

void SppLabelImage::CalculateClusterTopZ(const float* top_z_map) {
  for (auto& cluster : clusters_) {
    float sum = 0.f;
    for (auto& pixel : cluster->pixels) {
      sum += top_z_map[pixel];
    }
    sum = cluster->pixels.size() > 0
              ? sum / static_cast<float>(cluster->pixels.size())
              : sum;
    cluster->top_z = sum;
  }
}

void SppLabelImage::AddPixelSample(size_t id, uint32_t pixel) {
  if (clusters_.size() <= id) {
    SppClusterPool::Instance(sensor_name_)
        .BatchGet(id + 1 - clusters_.size(), &clusters_);
  }
  clusters_[id]->pixels.push_back(pixel);
}

void SppLabelImage::ResizeClusters(size_t size) {
  if (size > clusters_.size()) {
    SppClusterPool::Instance(sensor_name_)
        .BatchGet(size - clusters_.size(), &clusters_);
  } else {
    clusters_.resize(size);
  }
}

void SppLabelImage::ResetClusters(size_t size) {
  size_t reset_pos = std::min(clusters_.size(), size);
  ResizeClusters(size);
  for (size_t i = 0; i < reset_pos; ++i) {
    clusters_[i]->clear();
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
