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

#include "modules/perception/common/inference/inference_factory.h"

#include "modules/perception/common/inference/libtorch/torch_net.h"
#include "modules/perception/common/inference/onnx/libtorch_obstacle_detector.h"
#include "modules/perception/common/inference/onnx/onnx_single_batch_infer.h"
#include "modules/perception/common/inference/paddlepaddle/paddle_net.h"
#if GPU_PLATFORM == NVIDIA
#include "modules/perception/common/inference/tensorrt/rt_net.h"
#define RTNET RTNet(proto_file, weight_file, outputs, inputs)
#define RTNET8 RTNet(proto_file, weight_file, outputs, inputs, model_root)
#elif GPU_PLATFORM == AMD
#include "modules/perception/common/inference/migraphx/mi_net.h"
#define RTNET MINet(proto_file, weight_file, outputs, inputs)
// TODO(B1tway) Add quantization int8 support for RTNetInt8.
// RTNetInt8 on MIGraphX currently works with fp32.
#define RTNET8 RTNET
#endif

namespace apollo {
namespace perception {
namespace inference {

Inference *CreateInferenceByName(const std::string &frame_work,
                                 const std::string &proto_file,
                                 const std::string &weight_file,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs,
                                 const std::string &model_root) {
  if (frame_work == "RTNet") {
    return new RTNET;
  } else if (frame_work == "RTNetInt8") {
    return new RTNET8;
  } else if (frame_work == "TorchNet") {
    // PyTorch just have model file, we use proto_file as model file
    return new TorchNet(proto_file, outputs, inputs);
  } else if (frame_work == "Obstacle") {
    return new ObstacleDetector(proto_file, weight_file, outputs, inputs);
  } else if (frame_work == "Onnx") {
    return new SingleBatchInference(proto_file, outputs, inputs);
  } else if (frame_work == "PaddleNet") {
    return new PaddleNet(proto_file, weight_file, outputs, inputs);
  }
  return nullptr;
}

Inference *CreateInferenceByName(const common::Framework &frame_work,
                                 const std::string &proto_file,
                                 const std::string &weight_file,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs,
                                 const std::string &model_root) {
  switch (frame_work) {
    case common::TensorRT:
      if (model_root.empty()) {
        return new RTNET;
      }
      return new RTNET8;
    case common::PyTorch:
      return new TorchNet(proto_file, outputs, inputs);
    case common::PaddlePaddle:
      return new PaddleNet(proto_file, weight_file, outputs, inputs);
    case common::Obstacle:
      return new ObstacleDetector(proto_file, weight_file, outputs, inputs);
    case common::Onnx:
      return new SingleBatchInference(proto_file, outputs, inputs);
    default:
      break;
  }
  return nullptr;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
