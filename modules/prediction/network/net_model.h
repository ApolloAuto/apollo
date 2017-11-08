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

#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/prediction/network/net_layer.h"
#include "modules/prediction/network/net_util.h"
#include "modules/prediction/proto/network_model.pb.h"

#ifndef MODULES_PREDICTION_NETWORK_NET_MODEL_H_
#define MODULES_PREDICTION_NETWORK_NET_MODEL_H_

/**
 * @namespace apollo::prediction::network
 * @brief apollo::prediction::network
 */
namespace apollo {
namespace prediction {
namespace network {

class NetModel {
 public:
  NetModel();

  ~NetModel();

  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output) const = 0;

  virtual void SetState(const std::vector<Eigen::MatrixXf>& states) {}

  virtual void State(std::vector<Eigen::MatrixXf>* states) const {}

  virtual void ResetState() const {}

  bool LoadModel(const NetParameter& net_parameter);

  std::string PerformanceString() const;

  const std::string& Name() const;

  int Id() const;

  bool IsOk() const;

 protected:
  void Clear();

  std::vector<Layer*> layers_;
  NetParameter net_parameter_;
  bool ok_;
};

}  // namespace network
}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_NETWORK_NET_MODEL_H_
