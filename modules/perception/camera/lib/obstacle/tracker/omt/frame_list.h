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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/inference/utils/cuda_util.h"
#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace camera {

struct PatchIndicator {
  PatchIndicator() {
    frame_id = patch_id = -1;
    sensor_name = "";
  }

  PatchIndicator(int frame_id, int patch_id) {
    this->frame_id = frame_id;
    this->patch_id = patch_id;
    sensor_name = "";
  }
  PatchIndicator(int frame_id, int patch_id, const std::string &sensor_name) {
    this->frame_id = frame_id;
    this->patch_id = patch_id;
    this->sensor_name = sensor_name;
  }
  bool operator==(const PatchIndicator &indicator) {
    return (frame_id == indicator.frame_id && patch_id == indicator.patch_id);
  }

  std::string to_string() const {
    std::stringstream str;
    str << sensor_name << " | " << frame_id << " (" << patch_id << ")";
    return str.str();
  }

  int frame_id;
  int patch_id;
  std::string sensor_name;
};

struct SimilarMap {
 public:
  bool Init(int dim, int gpu_id = 0, int init_size = 128) {
    if (dim == 0) {
      return false;
    }

    inference::CudaUtil::set_device_id(gpu_id);

    this->dim = dim;
    map_sim.resize(dim);
    for (int i = 0; i < dim; ++i) {
      map_sim[i].resize(dim);
      for (int j = 0; j < dim; ++j) {
        map_sim[i][j].reset(new base::Blob<float>);
        map_sim[i][j]->Reshape({init_size, init_size});
        // allocate CPU/GPU memory
        map_sim[i][j]->cpu_data();
        map_sim[i][j]->gpu_data();
      }
    }
    return true;
  }

  void set(int frame1, int frame2, std::shared_ptr<base::Blob<float>> sim) {
    map_sim[frame1 % dim][frame2 % dim] = sim;
  }

  const std::shared_ptr<base::Blob<float>> get(int frame1, int frame2) const {
    return map_sim[frame1 % dim][frame2 % dim];
  }

  float sim(const PatchIndicator &p1, const PatchIndicator &p2) const {
    auto blob = get(p1.frame_id, p2.frame_id);
    return *(blob->cpu_data() + blob->offset(p1.patch_id, p2.patch_id));
  }

  std::vector<std::vector<std::shared_ptr<base::Blob<float>>>> map_sim;
  int dim;
};

class FrameList {
 public:
  FrameList() { Init(1); }

  bool Init(int cap) {
    if (cap <= 0) {
      return false;
    }
    capability_ = cap;
    frame_count_ = 0;
    frames_.resize(capability_);
    return true;
  }

  inline int OldestFrameId() {
    if (frame_count_ < capability_) {
      return 0;
    } else {
      return get_frame(frame_count_)->frame_id;
    }
  }

  inline int Size() {
    if (frame_count_ < capability_) {
      return frame_count_;
    } else {
      return capability_;
    }
  }

  inline void Add(CameraFrame *frame) {
    frames_[frame_count_ % capability_] = frame;
    ++frame_count_;
  }

  inline CameraFrame *get_frame(int index) const {
    assert(index <= frame_count_);
    if (index < 0) {
      return frames_[(index + frame_count_) % capability_];
    } else {
      return frames_[index % capability_];
    }
  }

  inline CameraFrame *operator[](int index) const { return get_frame(index); }

  inline base::ObjectPtr get_object(PatchIndicator indicator) const {
    return get_frame(indicator.frame_id)->detected_objects[indicator.patch_id];
  }

 private:
  int frame_count_ = 0;
  int capability_ = 0;
  std::vector<CameraFrame *> frames_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
