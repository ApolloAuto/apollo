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

#include "modules/perception/proto/rt.pb.h"

#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/tensorrt/entropy_calibrator.h"

namespace apollo {
namespace perception {
namespace inference {
class ArgMax1Plugin;
class ReLUPlugin;
class SLICEPlugin;
class SoftmaxPlugin;

typedef std::map<std::string, std::vector<nvinfer1::Weights>> WeightMap;
typedef std::map<std::string, nvinfer1::ITensor *> TensorMap;
typedef std::map<std::string, nvinfer1::DimsCHW> TensorDimsMap;
typedef std::map<std::string, std::string> TensorModifyMap;

const std::map<EltwiseParameter::EltwiseOp, nvinfer1::ElementWiseOperation>
    eltwise_map{
        {EltwiseParameter_EltwiseOp_PROD,
         nvinfer1::ElementWiseOperation::kPROD},
        {EltwiseParameter_EltwiseOp_SUM, nvinfer1::ElementWiseOperation::kSUM},
        {EltwiseParameter_EltwiseOp_MAX, nvinfer1::ElementWiseOperation::kMAX}};
const std::map<std::string, nvinfer1::ActivationType> active_map{
    {"Sigmoid", nvinfer1::ActivationType::kSIGMOID},
    {"TanH", nvinfer1::ActivationType::kTANH},
    {"ReLU", nvinfer1::ActivationType::kRELU}};
const std::vector<std::string> _gpu_checklist{
    "GeForce GTX 1080",    "GeForce GTX 1080 Ti", "Tesla P4",
    "Tesla P40",           "GeForce GTX 1070",    "GeForce GTX 1060",
    "Tesla V100-SXM2-16GB"};

class RTNet : public Inference {
 public:
  RTNet(const std::string &net_file, const std::string &model_file,
        const std::vector<std::string> &outputs,
        const std::vector<std::string> &inputs);
  RTNet(const std::string &net_file, const std::string &model_file,
        const std::vector<std::string> &outputs,
        const std::vector<std::string> &inputs,
        nvinfer1::Int8EntropyCalibrator *calibrator);
  RTNet(const std::string &net_file, const std::string &model_file,
        const std::vector<std::string> &outputs,
        const std::vector<std::string> &inputs, const std::string &model_root);

  virtual ~RTNet();

  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;

  void Infer() override;

  std::shared_ptr<apollo::perception::base::Blob<float>> get_blob(
      const std::string &name) override;

 protected:
  bool addInput(const TensorDimsMap &tensor_dims_map,
                const std::map<std::string, std::vector<int>> &shapes,
                TensorMap *tensor_map);
  bool shape(const std::string &name, std::vector<int> *res);

  void ConstructMap(const LayerParameter &layer_param, nvinfer1::ILayer *layer,
                    TensorMap *tensor_map, TensorModifyMap *tensor_modify_map);
  void parse_with_api(const std::map<std::string, std::vector<int>> &shapes);
  void addLayer(const LayerParameter &layer_param,
                nvinfer1::ITensor *const *inputs, int nbInputs,
                WeightMap *weight_map, nvinfer1::INetworkDefinition *net,
                TensorMap *tensor_map, TensorModifyMap *tensor_modify_map);

  void addConvLayer(const LayerParameter &layer_param,
                    nvinfer1::ITensor *const *inputs, WeightMap *weight_map,
                    nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                    TensorModifyMap *tensor_modify_map);

  void addDeconvLayer(const LayerParameter &layer_param,
                      nvinfer1::ITensor *const *inputs, WeightMap *weight_map,
                      nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addActiveLayer(const LayerParameter &layer_param,
                      nvinfer1::ITensor *const *inputs, int nbInputs,
                      nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addConcatLayer(const LayerParameter &layer_param,
                      nvinfer1::ITensor *const *inputs, int nbInputs,
                      nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addPoolingLayer(const LayerParameter &layer_param,
                       nvinfer1::ITensor *const *inputs,
                       nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);
  void addSliceLayer(const LayerParameter &layer_param,
                     nvinfer1::ITensor *const *inputs, int nbInputs,
                     nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                     TensorModifyMap *tensor_modify_map);

  void addInnerproductLayer(const LayerParameter &layer_param,
                            nvinfer1::ITensor *const *inputs,
                            WeightMap *weight_map,
                            nvinfer1::INetworkDefinition *net,
                            TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map);

  void addScaleLayer(const LayerParameter &layer_param,
                     nvinfer1::ITensor *const *inputs, WeightMap *weight_map,
                     nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                     TensorModifyMap *tensor_modify_map);

  void addBatchnormLayer(const LayerParameter &layer_param,
                         nvinfer1::ITensor *const *inputs,
                         WeightMap *weight_map,
                         nvinfer1::INetworkDefinition *net,
                         TensorMap *tensor_map,
                         TensorModifyMap *tensor_modify_map);

  void addSoftmaxLayer(const LayerParameter &layer_param,
                       nvinfer1::ITensor *const *inputs, int nbInputs,
                       nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addEltwiseLayer(const LayerParameter &layer_param,
                       nvinfer1::ITensor *const *inputs,
                       nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addArgmaxLayer(const LayerParameter &layer_param,
                      nvinfer1::ITensor *const *inputs, int nbInputs,
                      nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                      TensorModifyMap *tensor_modify_map);

  void addPermuteLayer(const LayerParameter &layer_param,
                       nvinfer1::ITensor *const *inputs, int nbInputs,
                       nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addReshapeLayer(const LayerParameter &layer_param,
                       nvinfer1::ITensor *const *inputs,
                       nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);

  void addPaddingLayer(const LayerParameter &layer_param,
                       nvinfer1::ITensor *const *inputs,
                       nvinfer1::INetworkDefinition *net, TensorMap *tensor_map,
                       TensorModifyMap *tensor_modify_map);
  bool checkInt8(const std::string &gpu_name,
                 nvinfer1::IInt8Calibrator *calibrator);
  void mergeBN(int index, LayerParameter *layer_param);
  nvinfer1::Weights loadLayerWeights(const float *data, int size);
  nvinfer1::Weights loadLayerWeights(float data, int size);

  bool loadWeights(const std::string &model_file, WeightMap *weight_map);
  void init_blob(std::vector<std::string> *names);

 private:
  nvinfer1::IExecutionContext *context_ = nullptr;
  cudaStream_t stream_ = 0;
  std::vector<std::shared_ptr<ArgMax1Plugin>> argmax_plugins_;
  std::vector<std::shared_ptr<SoftmaxPlugin>> softmax_plugins_;
  std::vector<std::shared_ptr<SLICEPlugin>> slice_plugins_;
  std::vector<std::shared_ptr<ReLUPlugin>> relu_plugins_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  std::map<std::string, std::string> tensor_modify_map_;

  std::shared_ptr<NetParameter> net_param_;
  WeightMap weight_map_;
  std::vector<void *> buffers_;
  int workspaceSize_ = 1;
  nvinfer1::Int8EntropyCalibrator *calibrator_ = nullptr;
  bool is_own_calibrator_ = true;
  std::string model_root_;
  nvinfer1::IBuilder *builder_ = nullptr;
  nvinfer1::INetworkDefinition *network_ = nullptr;
  std::vector<std::shared_ptr<float>> weights_mem_;
  BlobMap blobs_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
