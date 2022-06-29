#pragma once

#include <map>
#include <string>
#include <vector>

#include "modules/perception/proto/rt.pb.h"

#include "cyber/common/log.h"
#include "modules/perception/base/common.h"

namespace apollo {
namespace perception {
namespace inference {

typedef std::vector<int> TensorDims;
typedef std::map<std::string, TensorDims> TensorDimsMap;

void ParseNetParam(const NetParameter &net_param,
                   TensorDimsMap *tensor_dims_map,
                   std::map<std::string, std::string> *tensor_modify_map,
                   std::vector<LayerParameter> *order);

bool modify_pool_param(PoolingParameter *pool_param);

struct ConvParam {
  int kernel_h;
  int kernel_w;
  int padding_h;
  int padding_w;
  int stride_h;
  int stride_w;
  int group;
  int dilation;
};

bool ParserConvParam(const ConvolutionParameter &conv, ConvParam *param);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
