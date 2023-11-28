/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
 *
5 * Licensed under the Apache License, Version 2.0 (the "License");
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

#include "modules/perception/common/inference/migraphx/mi_net.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/perception/common/inference/migraphx/plugins/perception_inference_migraphx_plugins.h"

#define PRINT_DEBUG 1

#define DEBUG_GET_FN_NAME() \
  (LoggingParseFunction(__func__, __PRETTY_FUNCTION__))

#if PRINT_DEBUG
#define DEBUG_LOG(...)                                              \
  {                                                                 \
    AINFO << "INFO [" << DEBUG_GET_FN_NAME() << "] " << __VA_ARGS__ \
          << std::endl;                                             \
  }
#else
#define DEBUG_LOG(...) \
  {}
#endif

/// Expected to be invoked with __func__ and __PRETTY_FUNCTION__.
std::string LoggingParseFunction(const char *func, const char *pretty_func) {
  std::string fname{func};
  if (fname != "operator()") return fname;
  const std::string pf{pretty_func};
  const std::string pf_tail{pf.substr(0, pf.find_first_of('('))};
  return pf_tail.substr(1 + pf_tail.find_last_of(':'));
}

namespace apollo {
namespace perception {
namespace inference {

Dims GetDims(std::vector<size_t> lens, bool remove_batch = true) {
  Dims dims;

  // remove batch dimension for compatibility with Dims
  if (remove_batch) lens.erase(lens.begin());

  dims.nbDims = lens.size();
  for (size_t j = 0; j < lens.size(); j++) {
    dims.d[j] = lens[j];
  }

  return dims;
}

void MINet::ConstructMap(const LayerParameter &layer_param,
                         std::vector<Tensor> outputs, TensorMap *tensor_map,
                         TensorModifyMap *tensor_modify_map) {
  CHECK_EQ(outputs.size(), static_cast<size_t>(layer_param.top_size()));

  for (int i = 0; i < layer_param.top_size(); ++i) {
    std::string top_name = layer_param.top(i);
    TensorModifyMap::iterator it;
    it = tensor_modify_map->find(top_name);
    if (it != tensor_modify_map->end()) {
      std::string modify_name = top_name + "_" + layer_param.name();
      top_name = modify_name;
      it->second = top_name;
    } else {
      tensor_modify_map->insert(
          std::pair<std::string, std::string>(top_name, top_name));
    }

    const Tensor layer_out = outputs[i];

    if (std::find(output_names_.begin(), output_names_.end(), top_name) !=
        output_names_.end()) {
      const auto out_param_name =
          "main:#output_" + std::to_string(outputs_.size());
      const Tensor out_param =
          network_->add_parameter(out_param_name, layer_out->get_shape());
      const Tensor out = network_->add_instruction(
          migraphx::make_op("hip::copy"), layer_out, out_param);

      output_param_map_[top_name] = out_param_name;
      outputs_[top_name] = out;
    }

    DEBUG_LOG(top_name << ", " << layer_out->get_shape());
    tensor_map->insert(std::pair<std::string, Tensor>(top_name, layer_out));
  }
}

void MINet::addConvLayer(const LayerParameter &layer_param,
                         Tensor const *inputs, WeightMap *weight_map,
                         migraphx::module *net, TensorMap *tensor_map,
                         TensorModifyMap *tensor_modify_map) {
  const ConvolutionParameter conv_param = layer_param.convolution_param();
  ConvParam param;

  ACHECK(ParserConvParam(conv_param, &param));

  const auto input = inputs[0];
  const auto data_shape = input->get_shape();

  const size_t K = conv_param.num_output();
  const size_t C = data_shape.lens()[1] / param.group;
  const size_t R = param.kernel_h;
  const size_t S = param.kernel_w;

  const auto weights = (*weight_map)[layer_param.name().c_str()];
  const Shape filter_shape{Shape::float_type, {K, C, R, S}};

  CHECK_EQ(weights[0].bytes(), filter_shape.bytes());

  const Tensor filter =
      net->add_literal(migraphx::literal(filter_shape, weights[0].data()));

  const Operation conv = migraphx::make_op(
      "convolution", {{"padding", {param.padding_h, param.padding_w}},
                      {"stride", {param.stride_h, param.stride_w}},
                      {"dilation", {param.dilation, param.dilation}},
                      {"group", param.group}});
  Tensor out = net->add_instruction(conv, input, filter);

  if (weights.size() > 1) {
    const Shape out_shape = out->get_shape();
    const Shape bias_shape{Shape::float_type, {out_shape.lens()[1]}};

    CHECK_EQ(weights[1].bytes(), bias_shape.bytes());

    const Operation broadcast = migraphx::make_op(
        "broadcast", {{"axis", 1}, {"out_lens", out_shape.lens()}});
    const Tensor bias =
        net->add_literal(migraphx::literal(bias_shape, weights[1].data()));
    const Tensor broadcasted_bias = net->add_instruction(broadcast, bias);

    out = net->add_instruction(migraphx::make_op("add"), out, broadcasted_bias);
  }

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addDeconvLayer(const LayerParameter &layer_param,
                           Tensor const *inputs, WeightMap *weight_map,
                           migraphx::module *net, TensorMap *tensor_map,
                           TensorModifyMap *tensor_modify_map) {
  const ConvolutionParameter conv = layer_param.convolution_param();
  ConvParam param;

  ACHECK(ParserConvParam(conv, &param));

  const auto input = inputs[0];
  const auto data_shape = input->get_shape();

  const size_t K = data_shape.lens()[1] / param.group;
  const size_t C = conv.num_output();
  const size_t R = param.kernel_h;
  const size_t S = param.kernel_w;

  const auto weights = (*weight_map)[layer_param.name().c_str()];

  const Shape filter_shape{Shape::float_type, {K, C, R, S}};
  CHECK_EQ(weights[0].bytes(), filter_shape.bytes());

  const Tensor filter =
      net->add_literal(migraphx::literal(filter_shape, weights[0].data()));

  const Operation deconv = migraphx::make_op(
      "deconvolution", {{"padding", {param.padding_h, param.padding_w}},
                        {"stride", {param.stride_h, param.stride_w}},
                        {"dilation", {param.dilation, param.dilation}},
                        {"group", param.group}});

  Tensor out = net->add_instruction(deconv, input, filter);

  if (weights.size() > 1) {
    const Shape out_shape = out->get_shape();
    const Shape bias_shape{Shape::float_type, {out_shape.lens()[1]}};

    CHECK_EQ(weights[1].bytes(), bias_shape.bytes());

    const Operation broadcast = migraphx::make_op(
        "broadcast", {{"axis", 1}, {"out_lens", out_shape.lens()}});
    const Tensor bias =
        net->add_literal(migraphx::literal(bias_shape, weights[1].data()));
    const Tensor broadcasted_bias = net->add_instruction(broadcast, bias);

    out = net->add_instruction(migraphx::make_op("add"), out, broadcasted_bias);
  }

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addActiveLayer(const LayerParameter &layer_param,
                           Tensor const *inputs, int nbInputs,
                           migraphx::module *net, TensorMap *tensor_map,
                           TensorModifyMap *tensor_modify_map) {
  const auto input = inputs[0];

  if (layer_param.type() == "ReLU" &&
      layer_param.relu_param().negative_slope() > 0.0f) {
    const Operation leaky_relu = migraphx::make_op(
        "leaky_relu", {{"alpha", layer_param.relu_param().negative_slope()}});
    const Tensor out = net->add_instruction(leaky_relu, input);

    ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
  } else {
    // caffe default activation type is sigmoid
    std::string activation_type = "sigmoid";

    auto pair = active_map.find(layer_param.type());
    if (pair != active_map.end()) {
      activation_type = pair->second;
    }

    const Operation activation = migraphx::make_op(activation_type);
    const Tensor out = net->add_instruction(activation, input);

    ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
  }
}

void MINet::addConcatLayer(const LayerParameter &layer_param,
                           Tensor const *inputs, int nbInputs,
                           migraphx::module *net, TensorMap *tensor_map,
                           TensorModifyMap *tensor_modify_map) {
  const ConcatParameter params = layer_param.concat_param();
  // default caffe concat axis is 1
  const size_t axis = params.has_axis() ? params.axis() : 1;

  CHECK_EQ(nbInputs, layer_param.bottom_size());

  Tensor out = inputs[0];

  for (int i = 1; i < nbInputs; ++i) {
    const Tensor input_first = out;
    const Tensor input_second = inputs[i];
    const Shape first_shape = input_first->get_shape();
    const Shape second_shape = input_second->get_shape();

    CHECK_EQ(first_shape.lens().size(), second_shape.lens().size());
    for (size_t j = 0; j < first_shape.lens().size(); ++j) {
      if (j == axis) continue;
      CHECK_EQ(first_shape.lens()[j], second_shape.lens()[j]);
    }

    const Operation concat = migraphx::make_op("concat", {{"axis", axis}});
    out = net->add_instruction(concat, input_first, input_second);
  }

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addPoolingLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, migraphx::module *net,
                            TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map) {
  PoolingParameter params = layer_param.pooling_param();

  // dont support STOCHASTIC pooling mode yet
  CHECK_NE(params.pool(), PoolingParameter_PoolMethod_STOCHASTIC);

  // caffe default pooling mode is MAX
  const std::string pool_mode =
      (params.pool() == PoolingParameter_PoolMethod_AVE) ? "average" : "max";

  ACHECK(modify_pool_param(&params));

  const Operation pooling = migraphx::make_op(
      "pooling", {{"mode", pool_mode},
                  {"padding", {params.pad_h(), params.pad_w()}},
                  {"stride", {params.stride_h(), params.stride_w()}},
                  {"lengths", {params.kernel_h(), params.kernel_w()}}});
  const Tensor out = net->add_instruction(pooling, inputs[0]);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addSliceLayer(const LayerParameter &layer_param,
                          Tensor const *inputs, int nbInputs,
                          migraphx::module *net, TensorMap *tensor_map,
                          TensorModifyMap *tensor_modify_map) {
  const SliceParameter param = layer_param.slice_param();
  // default caffe concat axis is 1
  const size_t axis = param.has_axis() ? param.axis() : 1;

  const auto input = inputs[0];
  const auto input_shape = input->get_shape();

  CHECK_GT(param.slice_point_size(), 0);
  CHECK_GT(input_shape.lens().size(), axis);

  std::vector<Operation> ops;

  for (int i = 0; i < param.slice_point_size(); ++i) {
    const size_t slice_point = param.slice_point(i);

    if (i == 0) {
      ops.push_back(migraphx::make_op(
          "slice",
          {{"axes", {axis}}, {"starts", {0}}, {"ends", {slice_point}}}));
    } else {
      const size_t slice_point_prev = param.slice_point(i - 1);
      ops.push_back(migraphx::make_op("slice", {{"axes", {axis}},
                                                {"starts", {slice_point_prev}},
                                                {"ends", {slice_point}}}));
    }
  }

  const size_t last_slice_point =
      param.slice_point(param.slice_point_size() - 1);
  const size_t axis_size = input_shape.lens()[axis];

  ops.push_back(migraphx::make_op("slice", {{"axes", {axis}},
                                            {"starts", {last_slice_point}},
                                            {"ends", {axis_size}}}));

  std::vector<Tensor> outputs;
  for (auto &op : ops) {
    Tensor out = net->add_instruction(op, input);
    // keep standard NCHW layout
    out = net->add_instruction(migraphx::make_op("contiguous"), out);
    outputs.push_back(out);
  }

  ConstructMap(layer_param, outputs, tensor_map, tensor_modify_map);
}

void MINet::addInnerproductLayer(const LayerParameter &layer_param,
                                 Tensor const *inputs, WeightMap *weight_map,
                                 migraphx::module *net, TensorMap *tensor_map,
                                 TensorModifyMap *tensor_modify_map) {
  InnerProductParameter params = layer_param.inner_product_param();
  const size_t num_outputs = params.num_output();

  const auto input = inputs[0];
  const auto input_shape = input->get_shape();
  const auto input_lens = input_shape.lens();

  // Add FullyConnected layer as convolution
  const size_t K = num_outputs;
  const size_t C = input_lens[1];
  const size_t R = input_lens[2];
  const size_t S = input_lens[3];

  const Operation conv = migraphx::make_op("convolution", {{"padding", {0, 0}},
                                                           {"stride", {1, 1}},
                                                           {"dilation", {1, 1}},
                                                           {"group", 1}});

  const auto weights = (*weight_map)[layer_param.name().c_str()];
  const Shape filter_shape{Shape::float_type, {K, C, R, S}};

  CHECK_EQ(weights[0].bytes(), filter_shape.bytes());

  const Tensor filter =
      net->add_literal(migraphx::literal(filter_shape, weights[0].data()));

  Tensor out = net->add_instruction(conv, input, filter);

  if (weights.size() > 1) {
    const Shape out_shape = out->get_shape();
    const Shape bias_shape{Shape::float_type, {out_shape.lens()[1]}};

    CHECK_EQ(weights[1].bytes(), bias_shape.bytes());

    const Operation broadcast = migraphx::make_op(
        "broadcast", {{"axis", 1}, {"out_lens", out_shape.lens()}});
    const Tensor bias =
        net->add_literal(migraphx::literal(bias_shape, weights[1].data()));
    const Tensor broadcasted_bias = net->add_instruction(broadcast, bias);

    out = net->add_instruction(migraphx::make_op("add"), out, broadcasted_bias);
  }

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addScaleLayer(const LayerParameter &layer_param,
                          Tensor const *inputs, WeightMap *weight_map,
                          migraphx::module *net, TensorMap *tensor_map,
                          TensorModifyMap *tensor_modify_map) {
  const auto input = inputs[0];
  const auto input_shape = input->get_shape();
  const auto C = input_shape.lens()[1];

  std::vector<Weights> weights;

  if (layer_param.type() == "Power") {
    const auto power_param = layer_param.power_param();

    weights.push_back(loadLayerWeights(power_param.scale(), C));
    weights.push_back(loadLayerWeights(power_param.shift(), C));
    weights.push_back(loadLayerWeights(power_param.power(), C));
  } else {
    const auto lw = (*weight_map)[layer_param.name().c_str()];

    for (size_t i = 0; i < lw.size(); ++i) {
      weights.push_back(lw[i]);
    }
  }

  static const std::vector<std::string> scale_ops = {"mul", "add", "pow"};
  CHECK_LE(weights.size(), scale_ops.size());

  Tensor out = input;
  const Shape param_shape{Shape::float_type, {C}};

  for (size_t i = 0; i < weights.size(); ++i) {
    CHECK_EQ(weights[i].bytes(), param_shape.bytes());

    const Operation broadcast = migraphx::make_op(
        "broadcast", {{"axis", 1}, {"out_lens", input_shape.lens()}});
    const Tensor param =
        net->add_literal(migraphx::literal(param_shape, weights[i].data()));
    const Tensor broadcasted_param = net->add_instruction(broadcast, param);

    out = net->add_instruction(migraphx::make_op(scale_ops[i]), out,
                               broadcasted_param);
  }

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addBatchnormLayer(const LayerParameter &layer_param,
                              Tensor const *inputs, WeightMap *weight_map,
                              migraphx::module *net, TensorMap *tensor_map,
                              TensorModifyMap *tensor_modify_map) {
  const auto input = inputs[0];
  const auto input_shape = input->get_shape();
  const Shape params_shape{input_shape.type(), {input_shape.lens()[1]}};

  const auto weights = (*weight_map)[layer_param.name().c_str()];

  CHECK_EQ(weights[0].bytes(), params_shape.bytes());
  CHECK_EQ(weights[1].bytes(), params_shape.bytes());

  const Tensor shift =
      net->add_literal(migraphx::literal(params_shape, weights[0].data()));
  const Tensor scale =
      net->add_literal(migraphx::literal(params_shape, weights[1].data()));

  const Operation broadcast = migraphx::make_op(
      "broadcast", {{"axis", 1}, {"out_lens", input_shape.lens()}});

  const Tensor scale_broadcast = net->add_instruction(broadcast, scale);
  const Tensor mul_out =
      net->add_instruction(migraphx::make_op("mul"), input, scale_broadcast);

  const Tensor shift_broadcast = net->add_instruction(broadcast, shift);
  const Tensor out =
      net->add_instruction(migraphx::make_op("add"), mul_out, shift_broadcast);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addSoftmaxLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, int nbInputs,
                            migraphx::module *net, TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map) {
  const SoftmaxParameter params = layer_param.softmax_param();
  // default caffe softmax axis is 1
  size_t axis = params.has_axis() ? params.axis() : 1;
  // Workaround for baidu_iter_14000 model:
  // Batch dimension is omitted in the DFMBPSROIAlign output; due to specifics
  // of tensor definition in TensorRT it works correctly, but with MIGraphX,
  // the tensor dimension needs to be reduced.
  if (dfmb_psroi_align_plugins_.size() != 0) --axis;

  const auto input = inputs[0];
  const auto input_shape = input->get_shape();

  CHECK_GT(input_shape.lens().size(), axis);

  const Operation softmax = migraphx::make_op("softmax", {{"axis", axis}});
  const Tensor out = net->add_instruction(softmax, input);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addEltwiseLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, migraphx::module *net,
                            TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map) {
  // default caffe elemwise operation type is SUM
  std::string eltwise_op = "add";

  const auto pair = eltwise_map.find(layer_param.eltwise_param().operation());
  if (pair != eltwise_map.end()) {
    eltwise_op = pair->second;
  }

  const Operation eltwise = migraphx::make_op(eltwise_op);
  const Tensor out = net->add_instruction(eltwise, inputs[0], inputs[1]);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addArgmaxLayer(const LayerParameter &layer_param,
                           Tensor const *inputs, int nbInputs,
                           migraphx::module *net, TensorMap *tensor_map,
                           TensorModifyMap *tensor_modify_map) {
  const auto params = layer_param.argmax_param();
  const size_t axis = params.axis();

  // TODO(B1tway): Caffe also defines out_max_val and top_k argmax parameters,
  // but they are not supported yet
  ACHECK(!(params.has_out_max_val() || params.has_top_k()));

  const Tensor input = inputs[0];
  const Shape input_shape = input->get_shape();

  CHECK_GT(input_shape.lens().size(), axis);

  const Operation argmax = migraphx::make_op("argmax", {{"axis", axis}});
  const Tensor out = net->add_instruction(argmax, input);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addPermuteLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, int nbInputs,
                            migraphx::module *net, TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map) {
  const auto params = layer_param.permute_param();
  const auto input = inputs[0];
  const auto input_shape = input->get_shape();

  CHECK_EQ(static_cast<size_t>(params.order_size()), input_shape.lens().size());

  std::vector<uint32_t> order;
  for (int i = 0; i < params.order_size(); ++i) {
    const auto dim = params.order(i);
    CHECK_LT(dim, input_shape.lens().size());

    order.push_back(dim);
  }

  const Operation transpose =
      migraphx::make_op("transpose", {{"permutation", order}});
  const Tensor transposed_out = net->add_instruction(transpose, inputs[0]);

  // keep standard NCHW layout
  const Tensor out =
      net->add_instruction(migraphx::make_op("contiguous"), transposed_out);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addReshapeLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, migraphx::module *net,
                            TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map) {
  const auto params = layer_param.reshape_param();
  const auto input_shape = inputs[0]->get_shape();
  const auto input_lens = input_shape.lens();

  // For compatibility with tensorrt inference
  // skip batch size as it shouldn't be reshaped
  std::vector<int64_t> new_shape{int64_t(input_lens[0])};

  for (int i = 1; i < params.shape().dim_size(); ++i) {
    auto dim_param = params.shape().dim(i);

    // For compatibility with tensorrt inference reffer to:
    // void IShuffleLayer::setReshapeDimensions()
    // Ref: https://docs.nvidia.com/deeplearning/tensorrt/api/c_api

    int64_t new_dim = -1;
    if (dim_param == 0)
      new_dim = input_lens[i];
    else if (dim_param > 0)
      new_dim = dim_param;

    new_shape.push_back(new_dim);
  }

  const Operation reshape = migraphx::make_op("reshape", {{"dims", new_shape}});
  const Tensor out = net->add_instruction(reshape, inputs[0]);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addPaddingLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, migraphx::module *net,
                            TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map) {
  AWARN << "migraphx dont support Padding layer yet";
}

void MINet::addSilenceLayer(const LayerParameter &layer_param,
                            Tensor const *inputs, migraphx::module *net,
                            TensorMap *tensor_map,
                            TensorModifyMap *tensor_modify_map) {
  // default caffe elemwise operation type is SUM
  std::string eltwise_op = "add";

  for (int i = 0; i < layer_param.bottom_size(); ++i) {
    const auto name = layer_param.bottom(i);
    const Tensor tensor = tensor_map->at(tensor_modify_map_[name]);
    net->remove_instruction(tensor);
  }
}

void MINet::addDFMBPSROIAlignLayer(const LayerParameter &layer_param,
                                   Tensor const *inputs, int nbInputs,
                                   migraphx::module *net, TensorMap *tensor_map,
                                   TensorModifyMap *tensor_modify_map) {
  std::shared_ptr<DFMBPSROIAlignPlugin> plugin;
  std::vector<Dims> input_dims;

  CHECK_GE(nbInputs, 2);
  CHECK_LE(nbInputs, 3);
  CHECK_EQ(max_batch_size_, 1);

  for (int i = 0; i < nbInputs; ++i) {
    auto input_lens = inputs[i]->get_shape().lens();
    input_dims.push_back(GetDims(input_lens));
  }

  plugin.reset(new DFMBPSROIAlignPlugin(layer_param.dfmb_psroi_pooling_param(),
                                        input_dims.data(), nbInputs));
  dfmb_psroi_align_plugins_.push_back(plugin);

  // output dims without batchsize
  Dims out_dims = plugin->getOutputDimensions(0, nullptr, 0);
  Shape out_shape{Shape::float_type,
                  {std::size_t(out_dims.d[0]), std::size_t(out_dims.d[1]),
                   std::size_t(out_dims.d[2]), std::size_t(out_dims.d[3])}};

  const Tensor allocated_out = net->add_instruction(migraphx::make_op(
      "hip::allocate", {{"shape", migraphx::to_value(out_shape)}}));

  Tensor out;
  bool no_trans = nbInputs < 3;

  if (no_trans) {
    out =
        net->add_instruction(dfmb_psroi_align_op{size_t(max_batch_size_),
                                                 [plugin]() { return plugin; }},
                             inputs[0], inputs[1], allocated_out);
  } else {
    out =
        net->add_instruction(dfmb_psroi_align_op{size_t(max_batch_size_),
                                                 [plugin]() { return plugin; }},
                             inputs[0], inputs[1], inputs[2], allocated_out);
  }

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addRCNNProposalLayer(const LayerParameter &layer_param,
                                 Tensor const *inputs, int nbInputs,
                                 migraphx::module *net, TensorMap *tensor_map,
                                 TensorModifyMap *tensor_modify_map) {
  std::shared_ptr<RCNNProposalPlugin> plugin;
  std::vector<Dims> input_dims;

  CHECK_EQ(nbInputs, 4);

  for (int i = 0; i < nbInputs; ++i) {
    auto input_lens = inputs[i]->get_shape().lens();
    input_dims.push_back(GetDims(input_lens, i == 2 || i == 3));
  }

  plugin.reset(new RCNNProposalPlugin(layer_param.bbox_reg_param(),
                                      layer_param.detection_output_ssd_param(),
                                      input_dims.data()));
  rcnn_proposal_plugins_.push_back(plugin);

  // output dims without batchsize
  Dims out_dims = plugin->getOutputDimensions(0, nullptr, 0);
  Shape out_shape{Shape::float_type,
                  {std::size_t(max_batch_size_), std::size_t(out_dims.d[0]),
                   std::size_t(out_dims.d[1]), std::size_t(out_dims.d[2])}};

  const Tensor allocated_out = net->add_instruction(migraphx::make_op(
      "hip::allocate", {{"shape", migraphx::to_value(out_shape)}}));

  const Tensor out = net->add_instruction(
      rcnn_proposal_op{size_t(max_batch_size_), [plugin]() { return plugin; }},
      inputs[0], inputs[1], inputs[2], inputs[3], allocated_out);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addRPNProposalSSDLayer(const LayerParameter &layer_param,
                                   Tensor const *inputs, int nbInputs,
                                   migraphx::module *net, TensorMap *tensor_map,
                                   TensorModifyMap *tensor_modify_map) {
  std::shared_ptr<RPNProposalSSDPlugin> plugin;
  std::vector<Dims> input_dims;

  CHECK_EQ(nbInputs, 3);

  for (int i = 0; i < nbInputs; ++i) {
    auto input_lens = inputs[i]->get_shape().lens();
    Dims dims;

    // remove batch size for compatibility with Dims
    if (i == 0 || i == 1 || i == 2) {
      input_lens.erase(input_lens.begin());
    }

    for (size_t j = 0; j < input_lens.size(); ++j) {
      dims.d[j] = input_lens[j];
    }

    dims.nbDims = input_lens.size();
    input_dims.push_back(dims);
  }

  plugin.reset(new RPNProposalSSDPlugin(
      layer_param.bbox_reg_param(), layer_param.detection_output_ssd_param(),
      input_dims.data()));
  rpn_proposal_ssd_plugins_.push_back(plugin);

  // output dims without batchsize
  Dims out_dims = plugin->getOutputDimensions(0, nullptr, 0);
  Shape out_shape{
      Shape::float_type,
      {std::size_t(max_batch_size_ * out_dims.d[0]), std::size_t(out_dims.d[1]),
       std::size_t(out_dims.d[2]), std::size_t(out_dims.d[3])}};

  const Tensor allocated_out = net->add_instruction(migraphx::make_op(
      "hip::allocate", {{"shape", migraphx::to_value(out_shape)}}));

  const Tensor out =
      net->add_instruction(rpn_proposal_ssd_op{size_t(max_batch_size_),
                                               [plugin]() { return plugin; }},
                           inputs[0], inputs[1], inputs[2], allocated_out);

  ConstructMap(layer_param, {out}, tensor_map, tensor_modify_map);
}

void MINet::addLayer(const LayerParameter &layer_param, Tensor const *inputs,
                     int nbInputs, WeightMap *weight_map, migraphx::module *net,
                     TensorMap *tensor_map,
                     TensorModifyMap *tensor_modify_map) {
  DEBUG_LOG(layer_param.type());

  if (layer_param.type() == "Convolution") {
    addConvLayer(layer_param, inputs, weight_map, net, tensor_map,
                 tensor_modify_map);
  } else if (layer_param.type() == "Deconvolution") {
    addDeconvLayer(layer_param, inputs, weight_map, net, tensor_map,
                   tensor_modify_map);
  } else if (layer_param.type() == "Sigmoid" || layer_param.type() == "TanH" ||
             layer_param.type() == "ReLU") {
    addActiveLayer(layer_param, inputs, nbInputs, net, tensor_map,
                   tensor_modify_map);
  } else if (layer_param.type() == "Concat") {
    addConcatLayer(layer_param, inputs, nbInputs, net, tensor_map,
                   tensor_modify_map);
  } else if (layer_param.type() == "Slice") {
    addSliceLayer(layer_param, inputs, nbInputs, net, tensor_map,
                  tensor_modify_map);
  } else if (layer_param.type() == "Reshape") {
    addReshapeLayer(layer_param, inputs, net, tensor_map, tensor_modify_map);
  } else if (layer_param.type() == "Padding") {
    addPaddingLayer(layer_param, inputs, net, tensor_map, tensor_modify_map);
  } else if (layer_param.type() == "Pooling") {
    addPoolingLayer(layer_param, inputs, net, tensor_map, tensor_modify_map);
  } else if (layer_param.type() == "Permute") {
    addPermuteLayer(layer_param, inputs, nbInputs, net, tensor_map,
                    tensor_modify_map);
  } else if (layer_param.type() == "InnerProduct") {
    addInnerproductLayer(layer_param, inputs, weight_map, net, tensor_map,
                         tensor_modify_map);
  } else if (layer_param.type() == "BatchNorm") {
    addBatchnormLayer(layer_param, inputs, weight_map, net, tensor_map,
                      tensor_modify_map);
  } else if (layer_param.type() == "Scale") {
    addScaleLayer(layer_param, inputs, weight_map, net, tensor_map,
                  tensor_modify_map);
  } else if (layer_param.type() == "Softmax") {
    addSoftmaxLayer(layer_param, inputs, nbInputs, net, tensor_map,
                    tensor_modify_map);
  } else if (layer_param.type() == "Eltwise") {
    addEltwiseLayer(layer_param, inputs, net, tensor_map, tensor_modify_map);
  } else if (layer_param.type() == "ArgMax") {
    addArgmaxLayer(layer_param, inputs, nbInputs, net, tensor_map,
                   tensor_modify_map);
  } else if (layer_param.type() == "Dropout") {
    AINFO << "skip dropout";
  } else if (layer_param.type() == "Power") {
    addScaleLayer(layer_param, inputs, weight_map, net, tensor_map,
                  tensor_modify_map);
  } else if (layer_param.type() == "Silence") {
    addSilenceLayer(layer_param, inputs, net, tensor_map, tensor_modify_map);
  } else if (layer_param.type() == "DFMBPSROIAlign") {
    addDFMBPSROIAlignLayer(layer_param, inputs, nbInputs, net, tensor_map,
                           tensor_modify_map);
  } else if (layer_param.type() == "RCNNProposal") {
    addRCNNProposalLayer(layer_param, inputs, nbInputs, net, tensor_map,
                         tensor_modify_map);
  } else if (layer_param.type() == "RPNProposalSSD") {
    addRPNProposalSSDLayer(layer_param, inputs, nbInputs, net, tensor_map,
                           tensor_modify_map);
  } else {
    AWARN << "unknown layer type:" << layer_param.type();
  }
}

bool MINet::loadWeights(const std::string &model_file, WeightMap *weight_map) {
  NetParameter net;
  if (!ReadProtoFromBinaryFile(model_file.c_str(), &net)) {
    AFATAL << "open file " << model_file << " failed";
    return false;
  }
  for (int i = 0; i < net.layer_size(); ++i) {
    std::vector<Weights> lw;
    for (int j = 0; j < net.layer(i).blobs_size(); ++j) {
      // val memory will be released when deconstructor is called
      auto blob = &(net.layer(i).blobs(j));
      CHECK_EQ(blob->double_data_size(), 0);
      CHECK_EQ(blob->double_diff_size(), 0);
      CHECK_EQ(blob->diff_size(), 0);
      CHECK_NE(blob->data_size(), 0);
      if (net.layer(i).type() == "BatchNorm") {
        mergeBN(j, net.mutable_layer(i));
      }

      auto wt = loadLayerWeights(blob->data().data(), blob->data_size());
      lw.push_back(wt);
    }
    (*weight_map)[net.layer(i).name().c_str()] = lw;
  }
  return true;
}

void MINet::mergeBN(int index, LayerParameter *layer_param) {
  auto blob = (layer_param->mutable_blobs(index));
  CHECK_EQ(blob->double_data_size(), 0);
  CHECK_EQ(blob->double_diff_size(), 0);
  CHECK_EQ(blob->diff_size(), 0);
  CHECK_NE(blob->data_size(), 0);
  int size = blob->data_size();

  auto scale = (layer_param->blobs(2));
  const float scale_factor = scale.data(0) == 0 ? 0 : 1 / scale.data(0);
  auto epsilon = layer_param->batch_norm_param().eps();
  if (index == 0) {
    const BlobProto *var = (layer_param->mutable_blobs(index + 1));
    for (int k = 0; k < size; ++k) {
      auto data = blob->data(k);
      blob->set_data(
          k, static_cast<float>(-data * scale_factor /
                                sqrt(var->data(k) * scale_factor + epsilon)));
    }
  } else if (index == 1) {
    for (int k = 0; k < size; ++k) {
      blob->set_data(k, 1.0f / static_cast<float>(sqrt(
                                   blob->data(k) * scale_factor + epsilon)));
    }
  }
}

Weights MINet::loadLayerWeights(const float *data, size_t size) {
  std::shared_ptr<float> val;
  val.reset(new float[size]);

  for (size_t k = 0; k < size; ++k) {
    val.get()[k] = data[k];
  }

  Weights wt{size, val};
  return wt;
}

Weights MINet::loadLayerWeights(float data, size_t size) {
  std::shared_ptr<float> val;
  val.reset(new float[size]);

  for (size_t k = 0; k < size; ++k) {
    val.get()[k] = data;
  }

  Weights wt{size, val};
  return wt;
}

MINet::MINet(const std::string &net_file, const std::string &model_file,
             const std::vector<std::string> &outputs,
             const std::vector<std::string> &inputs)
    : output_names_(outputs), input_names_(inputs) {
  loadWeights(model_file, &weight_map_);
  net_param_.reset(new NetParameter);
  loadNetParams(net_file, net_param_.get());
}

MINet::MINet(const std::string &net_file, const std::string &model_file,
             const std::vector<std::string> &outputs,
             const std::vector<std::string> &inputs,
             const std::string &model_root)
    : output_names_(outputs), input_names_(inputs), model_root_(model_root) {
  loadWeights(model_file, &weight_map_);
  net_param_.reset(new NetParameter);
  loadNetParams(net_file, net_param_.get());
}

void MINet::init_blob(std::map<std::string, Tensor> *tensor_map) {
  for (const auto &p : *tensor_map) {
    auto name = p.first;
    auto shape = p.second->get_shape();

    cudaMalloc(&buffers_[name], shape.bytes());

    std::vector<size_t> dims_sizes = shape.lens();
    std::vector<int> dims(dims_sizes.begin(), dims_sizes.end());

    std::shared_ptr<apollo::perception::base::Blob<float>> blob;

    blob.reset(new apollo::perception::base::Blob<float>(dims));
    blob->set_gpu_data(reinterpret_cast<float *>(buffers_[name]));
    blobs_.insert(std::make_pair(name, blob));
  }
}

bool MINet::Init(const std::map<std::string, std::vector<int>> &shapes) {
  // TODO(B1tway): add cpu support
  if (gpu_id_ < 0) {
    AINFO << "must use gpu mode";
    return false;
  }

  auto target = migraphx::target(migraphx::gpu::target{});
  network_ = program_.get_main_module();

  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, gpu_id_);
  checkInt8(prop.name);

  parse_with_api(shapes);

  program_.compile(target);

#if PRINT_DEBUG
  {
    const auto output_shapes = program_.get_output_shapes();
    CHECK_EQ(outputs_.size(), output_shapes.size());

    std::stringstream ss;
    const auto shapes = program_.get_parameter_shapes();
    ss << "Parameter shapes: ";
    for (auto &p : shapes) {
      ss << p.first << " " << p.second << ", ";
    }
    ss << "\nOutput shapes: ";
    for (auto &o : output_shapes) {
      ss << o << ", ";
    }
    DEBUG_LOG(ss.str());
  }
#endif

  init_blob(&inputs_);
  init_blob(&outputs_);
  return true;
}

bool MINet::checkInt8(const std::string &gpu_name) {
  AWARN << "MIGraphX Inference Not Supports Int8 Mode. Use FP32 Mode.";
  return false;
}

bool MINet::addInput(TensorDimsMap *tensor_dims_map,
                     const std::map<std::string, std::vector<int>> &shapes,
                     TensorMap *tensor_map) {
  CHECK_GT(net_param_->layer_size(), 0);

  for (auto &dims_pair : *tensor_dims_map) {
    if (shapes.find(dims_pair.first) != shapes.end()) {
      auto shape = shapes.at(dims_pair.first);
      if (shape.size() == dims_pair.second.size()) {
        max_batch_size_ = std::max(max_batch_size_, shape[0]);
        for (size_t i = 0; i < dims_pair.second.size(); ++i) {
          dims_pair.second[i] = shape[i];
        }
      }
    }
    const Shape shape{Shape::float_type, dims_pair.second};
    const std::string name = dims_pair.first;
    const Tensor input = network_->add_parameter(name, shape);

    tensor_map->insert(std::make_pair(name, input));

    input_param_map_[name] = name;
    inputs_[name] = input;
  }

  return true;
}

void MINet::parse_with_api(
    const std::map<std::string, std::vector<int>> &shapes) {
  CHECK_GT(net_param_->layer_size(), 0);
  std::vector<LayerParameter> order;
  TensorMap tensor_map;
  TensorDimsMap tensor_dims_map;
  ParseNetParam(*net_param_, &tensor_dims_map, &tensor_modify_map_, &order);
  addInput(&tensor_dims_map, shapes, &tensor_map);

  for (auto layer_param : order) {
    std::vector<Tensor> inputs;
    for (int j = 0; j < layer_param.bottom_size(); ++j) {
      inputs.push_back(tensor_map[tensor_modify_map_[layer_param.bottom(j)]]);
    }
    addLayer(layer_param, inputs.data(), layer_param.bottom_size(),
             &weight_map_, network_, &tensor_map, &tensor_modify_map_);
  }

  std::vector<Tensor> outputs;
  std::transform(outputs_.begin(), outputs_.end(), std::back_inserter(outputs),
                 [](auto &p) { return p.second; });

  network_->add_return(outputs);
}

MINet::~MINet() {
  if (gpu_id_ >= 0) {
    for (auto buf : buffers_) {
      cudaFree(buf.second);
    }
  }
}

void MINet::Infer() {
  migraphx::gpu::set_device(gpu_id_);
  migraphx::gpu::gpu_sync();

  for (auto pair : input_param_map_) {
    const auto name = pair.first;
    const auto param_name = pair.second;
    const auto tensor = inputs_[name];
    auto blob = get_blob(name);
    if (blob != nullptr) {
      auto input_ptr = blob->mutable_gpu_data();
      auto input_shape = tensor->get_shape();
      parameter_map_[param_name] = migraphx::argument(input_shape, input_ptr);
    }
  }

  for (auto pair : output_param_map_) {
    const auto name = pair.first;
    const auto param_name = pair.second;
    const auto tensor = outputs_[name];
    auto blob = get_blob(name);
    if (blob != nullptr) {
      auto out_ptr = blob->mutable_gpu_data();
      auto out_shape = tensor->get_shape();
      parameter_map_[param_name] = migraphx::argument(out_shape, out_ptr);
    }
  }

  // Ensure input and output buffers are ready for inference
  migraphx::gpu::gpu_sync();

  auto output = program_.eval(parameter_map_);
  CHECK_EQ(output.size(), outputs_.size());
}

std::shared_ptr<apollo::perception::base::Blob<float>> MINet::get_blob(
    const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
