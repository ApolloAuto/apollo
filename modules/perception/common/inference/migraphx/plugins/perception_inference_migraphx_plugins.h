/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#include <migraphx/argument.hpp>
#include <migraphx/gpu/context.hpp>

#include "modules/perception/common/inference/migraphx/plugins/dfmb_psroi_align_plugin.h"
#include "modules/perception/common/inference/migraphx/plugins/rcnn_proposal_plugin.h"
#include "modules/perception/common/inference/migraphx/plugins/rpn_proposal_ssd_plugin.h"

namespace apollo {
namespace perception {
namespace inference {

struct dfmb_psroi_align_op {
  size_t batch_size;
  std::function<std::shared_ptr<DFMBPSROIAlignPlugin>()> get_plugin{};
  template <class Self, class F>
  static auto reflect(Self& self, F f) {  // NOLINT
    return migraphx::pack(f(self.batch_size, "batch_size"));
  }

  std::string name() const { return "dfmb_psroi_align"; }

  migraphx::shape compute_shape(
      const std::vector<migraphx::shape>& inputs) const {
    migraphx::check_shapes{inputs, *this}.standard();

    CHECK_EQ(static_cast<int>(batch_size), 1);

    auto plugin = get_plugin();
    auto out_dims = plugin->getOutputDimensions(0, nullptr, 0);

    return migraphx::shape{
        migraphx::shape::float_type,
        {std::size_t(out_dims.d[0]), std::size_t(out_dims.d[1]),
         std::size_t(out_dims.d[2]), std::size_t(out_dims.d[3])}};
  }

  migraphx::argument compute(
      migraphx::context& gctx,  // NOLINT
      const migraphx::shape&,
      const std::vector<migraphx::argument>& args) const {
    CHECK_GE(static_cast<int>(args.size()), 3);
    bool no_trans = args.size() < 4;  // 2 imputs + output

    auto bottom_data = args[0];
    auto bottom_rois = args[1];
    auto bottom_trans = no_trans ? migraphx::argument{} : args[2];
    auto top_data = no_trans ? args[2] : args[3];

    std::vector<const void*> inputs_data{
        bottom_data.data(), bottom_rois.data(),
        no_trans ? nullptr : bottom_trans.data()};

    std::vector<void*> outputs_data{top_data.data()};

    auto& ctx = migraphx::any_cast<migraphx::gpu::context>(gctx);

    auto plugin = get_plugin();
    plugin->enqueue(batch_size, inputs_data.data(), outputs_data.data(),
                    nullptr /*workspace*/, ctx.get_stream().get());
    return top_data;
  }

  int output_alias(const std::vector<migraphx::shape>& shapes) const {
    return shapes.size() - 1;
  }
};

struct rcnn_proposal_op {
  size_t batch_size;
  std::function<std::shared_ptr<RCNNProposalPlugin>()> get_plugin{};
  template <class Self, class F>
  static auto reflect(Self& self, F f) {  // NOLINT
    return migraphx::pack(f(self.batch_size, "batch_size"));
  }

  std::string name() const { return "rcnn_proposal"; }

  migraphx::shape compute_shape(
      const std::vector<migraphx::shape>& inputs) const {
    migraphx::check_shapes{inputs, *this}.has(5).standard();

    CHECK_EQ(inputs[3].lens()[0], batch_size);  // im_info

    auto plugin = get_plugin();
    auto out_dims = plugin->getOutputDimensions(0, nullptr, 0);

    return migraphx::shape{
        migraphx::shape::float_type,
        {std::size_t(batch_size), std::size_t(out_dims.d[0]),
         std::size_t(out_dims.d[1]), std::size_t(out_dims.d[2])}};
  }

  migraphx::argument compute(
      migraphx::context& gctx,  // NOLINT
      const migraphx::shape&,
      const std::vector<migraphx::argument>& args) const {
    std::vector<const void*> inputs_data{args[0].data(), args[1].data(),
                                         args[2].data(), args[3].data()};

    std::vector<void*> outputs_data{args[4].data()};

    auto& ctx = migraphx::any_cast<migraphx::gpu::context>(gctx);

    auto plugin = get_plugin();
    plugin->enqueue(batch_size, inputs_data.data(), outputs_data.data(),
                    nullptr /*workspace*/, ctx.get_stream().get());
    return args[4];
  }

  int output_alias(const std::vector<migraphx::shape>& shapes) const {
    return shapes.size() - 1;
  }
};

struct rpn_proposal_ssd_op {
  size_t batch_size;
  std::function<std::shared_ptr<RPNProposalSSDPlugin>()> get_plugin{};
  template <class Self, class F>
  static auto reflect(Self& self, F f) {  // NOLINT
    return migraphx::pack(f(self.batch_size, "batch_size"));
  }

  std::string name() const { return "rpn_proposal_ssd"; }

  migraphx::shape compute_shape(
      const std::vector<migraphx::shape>& inputs) const {
    migraphx::check_shapes{inputs, *this}.has(4).standard();

    CHECK_EQ(inputs[0].lens()[0], batch_size);  // rpn_cls_prob_reshape
    CHECK_EQ(inputs[1].lens()[0], batch_size);  // rpn_bbox_pred
    CHECK_EQ(inputs[2].lens()[0], batch_size);  // im_info

    auto plugin = get_plugin();
    auto out_dims = plugin->getOutputDimensions(0, nullptr, 0);

    return migraphx::shape{
        migraphx::shape::float_type,
        {std::size_t(batch_size), std::size_t(out_dims.d[0]),
         std::size_t(out_dims.d[1]), std::size_t(out_dims.d[2])}};
  }

  migraphx::argument compute(
      migraphx::context& gctx,  // NOLINT
      const migraphx::shape&,
      const std::vector<migraphx::argument>& args) const {
    std::vector<const void*> inputs_data{args[0].data(), args[1].data(),
                                         args[2].data()};

    std::vector<void*> outputs_data{args[3].data()};

    auto& ctx = migraphx::any_cast<migraphx::gpu::context>(gctx);

    auto plugin = get_plugin();
    plugin->enqueue(batch_size, inputs_data.data(), outputs_data.data(),
                    nullptr /*workspace*/, ctx.get_stream().get());
    return args[3];
  }

  int output_alias(const std::vector<migraphx::shape>& shapes) const {
    return shapes.size() - 1;
  }
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
