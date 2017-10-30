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

#include "modules/prediction/evaluator/network/net_model.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "modules/common/log.h"
#include "modules/prediction/evaluator/network/layer.h"
#include "modules/prediction/proto/network_model.pb.h"

namespace apollo {
namespace prediction {
namespace network {

NetModel::NetModel() : ok_(false) {}

NetModel::~NetModel() { Clear(); }

bool NetModel::LoadFromProtobuf(const std::string& model_filename) {
  std::fstream ifs(model_filename, std::ios::in | std::ios::binary);
  if (!ifs) {
    AERROR << "Fail to open file: " << model_filename.c_str();
    return false;
  }
  if (!net_parameter_.ParseFromIstream(&ifs)) {
    AERROR << "Fail to load model: " << model_filename.c_str();
    return false;
  }
  layers_.clear();

  for (int i = 0; i < net_parameter_.layers_size(); ++i) {
    LayerParameter layer_pb = net_parameter_.layers(i);
    ADEBUG << i << "-th layer name: " << layer_pb.name().c_str();
    Layer* layer = nullptr;
    switch (layer_pb.oneof_layers_case()) {
      case LayerParameter::kInput:
        layer = new Input();
        break;
      case LayerParameter::kActivation:
        layer = new Activation();
        break;
      case LayerParameter::kDense:
        layer = new Dense();
        break;
      case LayerParameter::kBatchNormalization:
        layer = new BatchNormalization();
        break;
      case LayerParameter::kLstm:
        layer = new LSTM();
        break;
      case LayerParameter::kFlatten:
        layer = new Flatten();
        break;
      case LayerParameter::kConcatenate:
        layer = new Concatenate();
        break;
      default:
        AERROR << "Fail to load layer: " << layer_pb.type().c_str();
        Clear();
        break;
    }
    if (!layer->Load(layer_pb)) {
      AERROR << "Fail to load " << i << "-layer: " << layer_pb.name().c_str();
      Clear();
      return false;
    }
    layers_.push_back(layer);
  }
  ok_ = true;
  AINFO << "Success to load model!";
  ADEBUG << "Its Performance:" << PerformanceString().c_str();
  return true;
}

void NetModel::Clear() {
  for (size_t i = 0; i < layers_.size(); ++i) {
    if (layers_[i] != nullptr) {
      delete layers_[i];
      layers_[i] = nullptr;
    }
  }
  ok_ = false;
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
