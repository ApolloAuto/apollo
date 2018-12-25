/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/network/net_model.h"

#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace prediction {
namespace network {

bool NetModel::LoadModel(const NetParameter& net_parameter) {
  net_parameter_.CopyFrom(net_parameter);
  layers_.clear();

  for (int i = 0; i < net_parameter_.layers_size(); ++i) {
    LayerParameter layer_pb = net_parameter_.layers(i);
    ADEBUG << i << "-th layer name: " << layer_pb.name().c_str();
    std::unique_ptr<Layer> layer(nullptr);
    switch (layer_pb.oneof_layers_case()) {
      case LayerParameter::kInput:
        layer = std::unique_ptr<Layer>(new Input());
        break;
      case LayerParameter::kActivation:
        layer = std::unique_ptr<Layer>(new Activation());
        break;
      case LayerParameter::kDense:
        layer = std::unique_ptr<Layer>(new Dense());
        break;
      case LayerParameter::kBatchNormalization:
        layer = std::unique_ptr<Layer>(new BatchNormalization());
        break;
      case LayerParameter::kLstm:
        layer = std::unique_ptr<Layer>(new LSTM());
        break;
      case LayerParameter::kFlatten:
        layer = std::unique_ptr<Layer>(new Flatten());
        break;
      case LayerParameter::kConcatenate:
        layer = std::unique_ptr<Layer>(new Concatenate());
        break;
      default:
        AERROR << "Failed to load layer: " << layer_pb.type().c_str();
        break;
    }
    if (!layer->Load(layer_pb)) {
      AERROR << "Failed to load " << i << "-layer: " << layer_pb.name().c_str();
      return false;
    }
    layers_.push_back(std::move(layer));
  }
  ok_ = true;
  AINFO << "Success in loading the model!";
  ADEBUG << "Its Performance:" << PerformanceString().c_str();
  return true;
}

std::string NetModel::PerformanceString() const {
  std::stringstream ss;
  if (!net_parameter_.has_performance()) {
    AWARN << "The protobuf does not contain performance values!";
    return " ";
  }
  ss << "\n test: accuracy = ";
  for (int i = 0; i < net_parameter_.performance().accuracy_size(); ++i) {
    ss << net_parameter_.performance().accuracy(i) << ",  ";
  }
  ss << "\n recall = ";
  for (int i = 0; i < net_parameter_.performance().recall_size(); ++i) {
    ss << net_parameter_.performance().recall(i) << ",  ";
  }
  ss << "\n precision = ";
  for (int i = 0; i < net_parameter_.performance().precision_size(); ++i) {
    ss << net_parameter_.performance().precision(i) << ",  ";
  }
  ss << std::endl;
  return ss.str();
}

const std::string& NetModel::Name() const { return net_parameter_.name(); }

int NetModel::Id() const { return net_parameter_.id(); }

bool NetModel::IsOk() const { return ok_; }

}  // namespace network
}  // namespace prediction
}  // namespace apollo
