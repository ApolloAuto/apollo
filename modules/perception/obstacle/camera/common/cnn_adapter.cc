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

#include "modules/perception/obstacle/camera/common/cnn_adapter.h"

#include <string>
#include <vector>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

// CNNCaffe
bool CNNCaffe::init(const std::vector<std::string> &input_names,
                    const std::vector<std::string> &output_names,
                    const std::string &proto_file,
                    const std::string &weight_file, int gpu_id,
                    const std::string &model_root) {
  ADEBUG << "proto_file: " << proto_file;
  ADEBUG << "weight_file: " << weight_file;
  ADEBUG << "model_root: " << model_root;

  if (gpu_id >= 0) {
    caffe::Caffe::SetDevice(gpu_id);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::DeviceQuery();
  } else {
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
  }

  // init Net
  net_.reset(new caffe::Net<float>(proto_file, caffe::TEST));
  CHECK((net_ != nullptr));
  net_->CopyTrainedLayersFrom(weight_file);
  gpu_id_ = gpu_id;
  return true;
}

void CNNCaffe::forward() { net_->Forward(); }

boost::shared_ptr<caffe::Blob<float>> CNNCaffe::get_blob_by_name(
    const std::string &name) {
  return net_->blob_by_name(name);
}
// CNNCaffe END

}  // namespace perception
}  // namespace apollo
