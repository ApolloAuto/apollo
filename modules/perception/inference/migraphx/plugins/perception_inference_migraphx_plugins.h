#pragma once

#include <memory>
#include <vector>

#include <migraphx/argument.hpp>
#include <migraphx/gpu/context.hpp>

#include "modules/perception/inference/migraphx/plugins/rcnn_proposal_plugin.h"

namespace apollo {
namespace perception {
namespace inference {

struct rcnn_proposal_op {
  std::shared_ptr<RCNNProposalPlugin> rcnn_proposal_plugin;

  template <class Self, class F>
  static auto reflect(Self& self, F f) {
    return migraphx::pack(
        f(*self.rcnn_proposal_plugin, "rcnn_proposal_plugin"));
  }

  std::string name() const { return "rcnn_proposal"; }

  migraphx::shape compute_shape(std::vector<migraphx::shape> inputs) const {
    std::vector<nvinfer1::Dims> input_dims;
    size_t batch_size;

    for (size_t i = 0; i < inputs.size(); i++) {
      auto lens = inputs[i].lens();
      nvinfer1::Dims dims;

      // nvinver1::Dims does not include batch size
      if (i == 3) {
        batch_size = lens[0];
        lens.erase(lens.begin());
      }

      for (size_t j = 0; j < lens.size(); j++) {
        dims.d[j] = lens[j];
      }
      dims.nbDims = lens.size();

      input_dims.push_back(dims);
    }

    auto out_dims =
        rcnn_proposal_plugin->getOutputDimensions(0, input_dims.data(), input_dims.size());

    return migraphx::shape{migraphx::shape::float_type,
                           {out_dims.d[0] * batch_size, out_dims.d[1],
                            out_dims.d[2], out_dims.d[3]}};
  }

  migraphx::argument compute(migraphx::context& gctx, const migraphx::shape&,
                             std::vector<migraphx::argument> args) const {
    std::vector<const void*> inputs_data{args[0].data(), args[1].data(),
                                         args[2].data(), args[3].data()};

    std::vector<void*> outputs_data{args[4].data()};

    // nvinver1::Dims does not include batch size
    auto im_info_shape = args[3].get_shape();
    auto im_info_dims = im_info_shape.lens();
    size_t batch_size = im_info_dims[0];

    auto& ctx = migraphx::any_cast<migraphx::gpu::context>(gctx);

    rcnn_proposal_plugin->enqueue(batch_size, inputs_data.data(),
                                  outputs_data.data(), nullptr /*workspace*/,
                                  ctx.get_stream().get());
    return args[4];
  }

  int output_alias(const std::vector<migraphx::shape>&) const { return 4; }
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo