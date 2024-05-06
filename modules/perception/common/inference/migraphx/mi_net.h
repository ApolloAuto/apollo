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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <migraphx/argument.hpp>
#include <migraphx/gpu/context.hpp>
#include <migraphx/gpu/hip.hpp>
#include <migraphx/gpu/target.hpp>
#include <migraphx/instruction.hpp>
#include <migraphx/make_op.hpp>
#include <migraphx/program.hpp>
#include <migraphx/ref/target.hpp>
#include <migraphx/target.hpp>

#include "modules/perception/common/proto/rt.pb.h"

#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/migraphx/mi_common.h"
#include "modules/perception/common/inference/migraphx/mi_utils.h"

namespace apollo {
namespace perception {
namespace inference {
class DFMBPSROIAlignPlugin;
class RCNNProposalPlugin;
class RPNProposalSSDPlugin;

struct Weights {
 private:
  size_t size_;
  std::shared_ptr<float> value_ptr_;

 public:
  Weights(size_t size, std::shared_ptr<float> value_ptr)
      : size_(size), value_ptr_(value_ptr) {}

  char *data() const { return reinterpret_cast<char *>(value_ptr_.get()); }
  size_t bytes() const { return size_ * sizeof(float); }
};

typedef migraphx::instruction_ref Tensor;
typedef migraphx::shape Shape;
typedef migraphx::operation Operation;
typedef std::map<std::string, std::vector<Weights>> WeightMap;
typedef std::map<std::string, Tensor> TensorMap;
typedef std::map<std::string, std::vector<int>> TensorDimsMap;
typedef std::map<std::string, std::string> TensorModifyMap;

const std::map<std::string, std::string> active_map{
    {"Sigmoid", "sigmoid"},
    {"TanH", "tanh"},
    {"ReLU", "relu"},
    {"LeakyReLU", "leaky_relu"}};

const std::map<EltwiseParameter::EltwiseOp, std::string> eltwise_map{
    {EltwiseParameter_EltwiseOp_PROD, "mul"},
    {EltwiseParameter_EltwiseOp_SUM, "add"},
    {EltwiseParameter_EltwiseOp_MAX, "max"}};

class MINet : public Inference {
 public:
  MINet(const std::string &net_file, const std::string &model_file,
        const std::vector<std::string> &outputs,
        const std::vector<std::string> &inputs);
  MINet(const std::string &net_file, const std::string &model_file,
        const std::vector<std::string> &outputs,
        const std::vector<std::string> &inputs, const std::string &model_root);

  virtual ~MINet();

  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;

  void Infer() override;

  std::shared_ptr<apollo::perception::base::Blob<float>> get_blob(
      const std::string &name) override;

 protected:
  bool addInput(TensorDimsMap *tensor_dims_map,
                const std::map<std::string, std::vector<int>> &shapes,
                TensorMap *tensor_map);

  void ConstructMap(const LayerParameter &layer_param,
                    std::vector<Tensor> outputs, TensorMap *tensor_map,
                    TensorModifyMap *tensor_modify_map);

  void parse_with_api(const std::map<std::string, std::vector<int>> &shapes);

  void addLayer(const LayerParameter &layer_param, Tensor const *inputs,
                int nbInputs, WeightMap *weight_map, migraphx::module *net,
                TensorMap *tensor_map, TensorModifyMap *tensor_modify_map);

  void addConvLayer(const LayerParameter &layer_param, Tensor const *inputs,
                    WeightMap *weight_map, migraphx::module *net,
                    TensorMap *tensor_map, TensorModifyMap *tensor_modify_map);

  void addDeconvLayer(const LayerParameter &layer_param, Tensor const *inputs,
                      WeightMap *weight_map, migraphx::module *net,
                      TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addActiveLayer(const LayerParameter &layer_param, Tensor const *inputs,
                      int nbInputs, migraphx::module *net,
                      TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addConcatLayer(const LayerParameter &layer_param, Tensor const *inputs,
                      int nbInputs, migraphx::module *net,
                      TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addPoolingLayer(const LayerParameter &layer_param, Tensor const *inputs,
                       migraphx::module *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addSliceLayer(const LayerParameter &layer_param, Tensor const *inputs,
                     int nbInputs, migraphx::module *net, TensorMap *tensor_map,
                     TensorModifyMap *tensor_modify_map);

  void addInnerproductLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, WeightMap *weight_map,
                            migraphx::module *net, TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map);

  void addScaleLayer(const LayerParameter &layer_param, Tensor const *inputs,
                     WeightMap *weight_map, migraphx::module *net,
                     TensorMap *tensor_map, TensorModifyMap *tensor_modify_map);

  void addBatchnormLayer(const LayerParameter &layer_param,
                         Tensor const *inputs, WeightMap *weight_map,
                         migraphx::module *net, TensorMap *tensor_map,
                         TensorModifyMap *tensor_modify_map);

  void addSoftmaxLayer(const LayerParameter &layer_param, Tensor const *inputs,
                       int nbInputs, migraphx::module *net,
                       TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addEltwiseLayer(const LayerParameter &layer_param, Tensor const *inputs,
                       migraphx::module *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addArgmaxLayer(const LayerParameter &layer_param, Tensor const *inputs,
                      int nbInputs, migraphx::module *net,
                      TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addPermuteLayer(const LayerParameter &layer_param, Tensor const *inputs,
                       int nbInputs, migraphx::module *net,
                       TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addReshapeLayer(const LayerParameter &layer_param, Tensor const *inputs,
                       migraphx::module *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addPaddingLayer(const LayerParameter &layer_param, Tensor const *inputs,
                       migraphx::module *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addSilenceLayer(const LayerParameter &layer_param, Tensor const *inputs,
                       migraphx::module *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addDFMBPSROIAlignLayer(const LayerParameter &layer_param,
                              Tensor const *inputs, int nbInputs,
                              migraphx::module *net, TensorMap *tensor_map,
                              TensorModifyMap *tensor_modify_map);

  void addRCNNProposalLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, int nbInputs,
                            migraphx::module *net, TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map);

  void addRPNProposalSSDLayer(const LayerParameter &layer_param,
                              Tensor const *inputs, int nbInputs,
                              migraphx::module *net, TensorMap *tensor_map,
                              TensorModifyMap *tensor_modify_map);

  bool checkInt8(const std::string &gpu_name);

  void mergeBN(int index, LayerParameter *layer_param);
  Weights loadLayerWeights(const float *data, size_t size);
  Weights loadLayerWeights(float data, size_t size);

  bool loadWeights(const std::string &model_file, WeightMap *weight_map);
  void init_blob(std::map<std::string, Tensor> *tensors);

 private:
  std::vector<std::shared_ptr<DFMBPSROIAlignPlugin>> dfmb_psroi_align_plugins_;
  std::vector<std::shared_ptr<RCNNProposalPlugin>> rcnn_proposal_plugins_;
  std::vector<std::shared_ptr<RPNProposalSSDPlugin>> rpn_proposal_ssd_plugins_;

  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  std::map<std::string, Tensor> outputs_;
  std::map<std::string, Tensor> inputs_;
  std::map<std::string, std::string> tensor_modify_map_;
  std::map<std::string, std::string> input_param_map_;
  std::map<std::string, std::string> output_param_map_;

  migraphx::module *network_;
  migraphx::program program_;
  migraphx::parameter_map parameter_map_;

  std::shared_ptr<NetParameter> net_param_;
  WeightMap weight_map_;
  std::map<std::string, void *> buffers_;
  std::string model_root_;
  BlobMap blobs_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
