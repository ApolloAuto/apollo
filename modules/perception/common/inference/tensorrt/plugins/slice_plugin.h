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

#include <NvInferVersion.h>

#include <algorithm>
#include <string>
#include <vector>

#include "modules/perception/common/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR != 8
class SLICEPlugin : public nvinfer1::IPlugin {
 public:
  SLICEPlugin(const SliceParameter &param, const nvinfer1::Dims &in_dims) {
    CHECK_GT(param.slice_point_size(), 0);
    for (int i = 0; i < param.slice_point_size(); i++) {
      slice_point_.push_back(param.slice_point(i));
    }
    axis_ = std::max(param.axis() - 1, 0);
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      input_dims_.type[i] = in_dims.type[i];
    }

    for (size_t i = 0; i < slice_point_.size(); i++) {
      if (i == 0) {
        out_slice_dims_.push_back(slice_point_[i]);
      } else {
        out_slice_dims_.push_back(slice_point_[i] - slice_point_[i - 1]);
      }
    }
    out_slice_dims_.push_back(input_dims_.d[axis_] -
                              slice_point_[slice_point_.size() - 1]);
  }
  SLICEPlugin() {}
  ~SLICEPlugin() {}
  virtual int initialize() { return 0; }
  virtual void terminate() {}
  int getNbOutputs() const override {
    return static_cast<int>(slice_point_.size()) + 1;
  }
  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) override {
    nvinfer1::Dims out_dims = inputs[0];
    out_dims.d[axis_] = out_slice_dims_[index];
    return out_dims;
  }

  void configure(const nvinfer1::Dims *inputDims, int nbInputs,
                 const nvinfer1::Dims *outputDims, int nbOutputs,
                 int maxBatchSize) override {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int maxBatchSize) const override { return 0; }

  virtual int enqueue(int batchSize, const void *const *inputs, void **outputs,
                      void *workspace, cudaStream_t stream);

  size_t getSerializationSize() override { return 0; }

  void serialize(void *buffer) override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

 private:
  std::vector<int> slice_point_;
  std::vector<int> out_slice_dims_;
  int axis_;
  nvinfer1::Dims input_dims_;
};

#else
class SLICEPlugin : public nvinfer1::IPluginV2 {
 public:
  SLICEPlugin(const char *layer_name, const void *serial_data,
              size_t serial_length) {
    const char *p = reinterpret_cast<const char *>(serial_data);

    input_dims_.nbDims = reinterpret_cast<const int *>(p)[0];
    CHECK_GT(input_dims_.nbDims, 0);
    p += sizeof(int);
    for (int i = 0; i < input_dims_.nbDims; ++i) {
      input_dims_.d[i] = reinterpret_cast<const int *>(p)[0];
      p += sizeof(int);
    }

    {
      size_t size = reinterpret_cast<const size_t *>(p)[0];
      CHECK_GT(size, 0);
      p += sizeof(size_t);
      slice_point_.assign(reinterpret_cast<const int *>(p),
                          reinterpret_cast<const int *>(p) + size);
      p += sizeof(int) * size;
    }

    {
      size_t size = reinterpret_cast<const size_t *>(p)[0];
      CHECK(size == slice_point_.size() + 1);
      p += sizeof(size_t);
      out_slice_dims_.assign(reinterpret_cast<const int *>(p),
                             reinterpret_cast<const int *>(p) + size);
      p += sizeof(int) * size;
    }

    axis_ = reinterpret_cast<const int *>(p)[0];
    p += sizeof(int);

    CHECK(p == reinterpret_cast<const char *>(serial_data) + serial_length);
    CHECK(getSerializationSize() == serial_length);
  }

  SLICEPlugin(const SliceParameter &param, const nvinfer1::Dims &in_dims) {
    CHECK_GT(param.slice_point_size(), 0);
    for (int i = 0; i < param.slice_point_size(); i++) {
      slice_point_.push_back(param.slice_point(i));
    }
    axis_ = std::max(param.axis() - 1, 0);
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
    }

    for (size_t i = 0; i < slice_point_.size(); i++) {
      if (i == 0) {
        out_slice_dims_.push_back(slice_point_[i]);
      } else {
        out_slice_dims_.push_back(slice_point_[i] - slice_point_[i - 1]);
      }
    }
    out_slice_dims_.push_back(input_dims_.d[axis_] -
                              slice_point_[slice_point_.size() - 1]);
  }

  SLICEPlugin(const int axis, const std::vector<int> &slice_point,
              const nvinfer1::Dims &in_dims) {
    CHECK_GT(slice_point.size(), 0);
    for (size_t i = 0; i < slice_point.size(); i++) {
      slice_point_.push_back(slice_point[i]);
    }
    axis_ = std::max(axis - 1, 0);
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
    }

    for (size_t i = 0; i < slice_point_.size(); i++) {
      if (i == 0) {
        out_slice_dims_.push_back(slice_point_[i]);
      } else {
        out_slice_dims_.push_back(slice_point_[i] - slice_point_[i - 1]);
      }
    }
    out_slice_dims_.push_back(input_dims_.d[axis_] -
                              slice_point_[slice_point_.size() - 1]);
  }
  SLICEPlugin() {}
  ~SLICEPlugin() {}
  virtual int initialize() noexcept { return 0; }

  virtual void terminate() noexcept {}

  int getNbOutputs() const noexcept override { return slice_point_.size() + 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    nvinfer1::Dims out_dims = inputs[0];
    out_dims.d[axis_] = out_slice_dims_[index];
    return out_dims;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims, int nbInputs,
                           const nvinfer1::Dims *outputDims, int nbOutputs,
                           nvinfer1::DataType type,
                           nvinfer1::PluginFormat format,
                           int maxBatchSize) noexcept override {
    input_dims_ = inputDims[0];
  }

  void configure(const nvinfer1::Dims *inputDims, int nbInputs,
                 const nvinfer1::Dims *outputDims, int nbOutputs,
                 int maxBatchSize) {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int maxBatchSize) const noexcept override {
    return 0;
  }

  virtual int enqueue(int batchSize, void const *const *inputs,
                      void *const *outputs, void *workspace,
                      cudaStream_t stream) noexcept;

  size_t getSerializationSize() const noexcept override {
    /* return sizeof(size_t) + Name().size() + \ */
    return sizeof(int) + sizeof(int) * 1 * input_dims_.nbDims + sizeof(size_t) +
           slice_point_.size() * sizeof(slice_point_[0]) + sizeof(size_t) +
           out_slice_dims_.size() * sizeof(out_slice_dims_[0]) + sizeof(axis_);
  }

  void serialize(void *buffer) const noexcept override {
    char *p = reinterpret_cast<char *>(buffer);
    // setPluginType(&p, Name());

    reinterpret_cast<int *>(p)[0] = input_dims_.nbDims;
    p += sizeof(int);
    for (int i = 0; i < input_dims_.nbDims; ++i) {
      reinterpret_cast<int *>(p)[0] = input_dims_.d[i];
      p += sizeof(int);
      // reinterpret_cast<int *>(p)[0] = static_cast<int>(input_dims_.type[i]);
      // p += sizeof(int);
    }

    reinterpret_cast<size_t *>(p)[0] = slice_point_.size();
    p += sizeof(size_t);
    for (size_t i = 0; i < slice_point_.size(); ++i) {
      reinterpret_cast<int *>(p)[0] = slice_point_[i];
      p += sizeof(int);
    }

    reinterpret_cast<size_t *>(p)[0] = out_slice_dims_.size();
    p += sizeof(size_t);
    for (size_t i = 0; i < out_slice_dims_.size(); ++i) {
      reinterpret_cast<int *>(p)[0] = out_slice_dims_[i];
      p += sizeof(int);
    }

    reinterpret_cast<int *>(p)[0] = axis_;
    p += sizeof(int);
  }

  std::string Name() const { return "slice_plugin"; }

  const char *getPluginType() const noexcept override {
    return "slice_plugin";
  };

  const char *getPluginVersion() const noexcept override { return "1"; }

  bool supportsFormat(nvinfer1::DataType type,
                      nvinfer1::PluginFormat format) const noexcept override {
    // if (format != PluginFormat::kNCHW) {
    //   return false;
    // }
    // if (type == nvinfer1::DataType::kINT32 ||
    //     type == nvinfer1::DataType::kINT8) {
    //   return false;
    // }
    return true;
  }

  void destroy() noexcept override {}

  IPluginV2 *clone() const noexcept override {
    return new SLICEPlugin(axis_ + 1, slice_point_, input_dims_);
  }

  const char *getPluginNamespace() const noexcept override { return "camera"; }

  void setPluginNamespace(const char *s_name) noexcept override {
    name_ = s_name;
  }

 private:
  std::vector<int> slice_point_;
  std::vector<int> out_slice_dims_;
  int axis_;
  const char *name_ = "camera";
  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
};

class SLICEPluginCreator : public nvinfer1::IPluginCreator {
 public:
  SLICEPluginCreator() {}
  ~SLICEPluginCreator() {
    // terminae();
  }
  const char *getPluginName() const noexcept override {
    return "slice_plugincamera";
  }
  const char *getPluginVersion() const noexcept override { return "1"; }

  const nvinfer1::PluginFieldCollection *getFiledNames() { return nullptr; }
  nvinfer1::IPluginV2 *createPlugin(
      const char *name,
      const nvinfer1::PluginFieldCollection *fc) noexcept override {
    int axis;
    std::vector<int> slice_point;
    nvinfer1::Dims in_dims;
    const nvinfer1::PluginField *fields = fc->fields;
    for (int i = 0; i < fc->nbFields; ++i) {
      const char *attrName = fields[i].name;
      if (!strcmp(attrName, "axis")) {
        // ASSERT(fields[i].type == nvinfer1::PluginFieldType::kINT32);
        axis = *(static_cast<const int32_t *>(fields[i].data));
      } else if (!strcmp(attrName, "slice_point")) {
        // ASSERT(fields[i].type == nvinfer1::PluginFieldType::kINT32);
        int num = fields[i].length;
        slice_point.reserve(num);
        const auto w = static_cast<const int32_t *>(fields[i].data);
        for (int j = 0; j < num; ++j) {
          slice_point.push_back(w[j]);
        }
      } else if (!strcmp(attrName, "in_dims")) {
        // ASSERT(fields[i].type ==nvinfer1::PluginFieldType::kDIMS);
        in_dims = *(static_cast<const nvinfer1::Dims *>(fields[i].data));
      }
    }
    SLICEPlugin *obj = new SLICEPlugin(axis, slice_point, in_dims);
    obj->setPluginNamespace(mNamespaces.c_str());
    return obj;
  }

  nvinfer1::IPluginV2 *deserializePlugin(
      const char *name, const void *serial_data,
      size_t serial_length) noexcept override {
    return new SLICEPlugin(name, serial_data, serial_length);
  }
  const char *getPluginNamespace() const noexcept override { return "camera"; }

  void setPluginNamespace(const char *s_name) noexcept override {
    mNamespaces = s_name;
  }
  const nvinfer1::PluginFieldCollection *getFieldNames() noexcept override {
    return nullptr;
  }

 private:
  std::string mNamespaces = "camera";
};

REGISTER_TENSORRT_PLUGIN(SLICEPluginCreator);
#endif
#endif
}  // namespace inference
}  // namespace perception
}  // namespace apollo
