/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// headers in STL
#include <chrono>
#include <iostream>

// headers in local files
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.h"

namespace apollo {
namespace perception {
namespace lidar {

PointPillars::PointPillars(const bool reproduce_result_mode,
                           const int num_class,
                           const float score_threshold,
                           const float nms_overlap_threshold,
                           const std::string pfe_onnx_file,
                           const std::string rpn_onnx_file)
    : kReproduceResultMode(reproduce_result_mode),
      kScoreThreshold(score_threshold),
      kNmsOverlapThreshold(nms_overlap_threshold),
      kPfeOnnxFile(pfe_onnx_file),
      kRpnOnnxFile(rpn_onnx_file),
      kMaxNumPillars(12000),
      kMaxNumPointsPerPillar(100),
      kPfeOutputSize(kMaxNumPillars * 64),
      kGridXSize(432),
      kGridYSize(496),
      kGridZSize(1),
      kRpnInputSize(64 * kGridXSize * kGridYSize),
      // TODO(chenjiahao): Enable customizing sizes of multiple anchors
      kNumAnchorXInds(kGridXSize * 0.5),
      kNumAnchorYInds(kGridYSize * 0.5),
      kNumAnchorRInds(2),
      kNumAnchor(kNumAnchorXInds * kNumAnchorYInds * kNumAnchorRInds),
      // TODO(chenjiahao): Should be defined by the input param num_class
      kNumClass(3),
      kRpnBoxOutputSize(kNumAnchor * 7),
      kRpnClsOutputSize(kNumAnchor * kNumClass),
      kRpnDirOutputSize(kNumAnchor * 2),
      kPillarXSize(0.16f),
      kPillarYSize(0.16f),
      kPillarZSize(4.0f),
      kMinXRange(0.0f),
      kMinYRange(-39.68f),
      kMinZRange(-3.0f),
      kMaxXRange(69.12f),
      kMaxYRange(39.68f),
      kMaxZRange(1),
      kBatchSize(1),
      kNumIndsForScan(512),
      kNumThreads(64)
      // if you change kNumThreads, need to modify NUM_THREADS_MACRO in
      // common.h
      ,
      kSensorHeight(1.73f),
      kAnchorDxSize(1.6f),
      kAnchorDySize(3.9f),
      kAnchorDzSize(1.56f),
      kNumBoxCorners(4),
      kNumOutputBoxFeature(7) {
  if (kReproduceResultMode) {
    preprocess_points_ptr_.reset(new PreprocessPoints(
        kMaxNumPillars, kMaxNumPointsPerPillar, kGridXSize,
        kGridYSize, kGridZSize, kPillarXSize, kPillarYSize,
        kPillarZSize, kMinXRange, kMinYRange, kMinZRange,
        kNumIndsForScan, kNumBoxCorners));
  } else {
    preprocess_points_cuda_ptr_.reset(new PreprocessPointsCuda(
        kNumThreads, kMaxNumPillars, kMaxNumPointsPerPillar,
        kNumIndsForScan, kGridXSize, kGridYSize, kGridZSize,
        kPillarXSize, kPillarYSize, kPillarZSize, kMinXRange,
        kMinYRange, kMinZRange, kNumBoxCorners));
  }

  anchor_mask_cuda_ptr_.reset(new AnchorMaskCuda(
      kNumIndsForScan, kNumAnchorXInds, kNumAnchorYInds,
      kNumAnchorRInds, kMinXRange, kMinYRange, kPillarXSize,
      kPillarYSize, kGridXSize, kGridYSize));

  scatter_cuda_ptr_.reset(new ScatterCuda(kNumThreads, kMaxNumPillars,
                                          kGridXSize, kGridYSize));

  const float float_min = std::numeric_limits<float>::lowest();
  const float float_max = std::numeric_limits<float>::max();
  postprocess_cuda_ptr_.reset(new PostprocessCuda(
      float_min, float_max, kNumAnchorXInds, kNumAnchorYInds,
      kNumAnchorRInds, kScoreThreshold, kNumThreads,
      kNmsOverlapThreshold, kNumBoxCorners, kNumOutputBoxFeature,
      kNumClass));

  DeviceMemoryMalloc();
  InitTRT();
  InitAnchors();
}

PointPillars::~PointPillars() {
  delete[] anchors_px_;
  delete[] anchors_py_;
  delete[] anchors_pz_;
  delete[] anchors_dx_;
  delete[] anchors_dy_;
  delete[] anchors_dz_;
  delete[] anchors_ro_;
  delete[] box_anchors_min_x_;
  delete[] box_anchors_min_y_;
  delete[] box_anchors_max_x_;
  delete[] box_anchors_max_y_;

  GPU_CHECK(cudaFree(dev_pillar_x_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_y_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_z_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_i_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_count_histo_));

  GPU_CHECK(cudaFree(dev_x_coors_));
  GPU_CHECK(cudaFree(dev_y_coors_));
  GPU_CHECK(cudaFree(dev_num_points_per_pillar_));
  GPU_CHECK(cudaFree(dev_sparse_pillar_map_));

  GPU_CHECK(cudaFree(dev_pillar_x_));
  GPU_CHECK(cudaFree(dev_pillar_y_));
  GPU_CHECK(cudaFree(dev_pillar_z_));
  GPU_CHECK(cudaFree(dev_pillar_i_));

  GPU_CHECK(cudaFree(dev_x_coors_for_sub_shaped_));
  GPU_CHECK(cudaFree(dev_y_coors_for_sub_shaped_));
  GPU_CHECK(cudaFree(dev_pillar_feature_mask_));

  GPU_CHECK(cudaFree(dev_cumsum_along_x_));
  GPU_CHECK(cudaFree(dev_cumsum_along_y_));

  GPU_CHECK(cudaFree(dev_box_anchors_min_x_));
  GPU_CHECK(cudaFree(dev_box_anchors_min_y_));
  GPU_CHECK(cudaFree(dev_box_anchors_max_x_));
  GPU_CHECK(cudaFree(dev_box_anchors_max_y_));
  GPU_CHECK(cudaFree(dev_anchor_mask_));

  GPU_CHECK(cudaFree(pfe_buffers_[0]));
  GPU_CHECK(cudaFree(pfe_buffers_[1]));
  GPU_CHECK(cudaFree(pfe_buffers_[2]));
  GPU_CHECK(cudaFree(pfe_buffers_[3]));
  GPU_CHECK(cudaFree(pfe_buffers_[4]));
  GPU_CHECK(cudaFree(pfe_buffers_[5]));
  GPU_CHECK(cudaFree(pfe_buffers_[6]));
  GPU_CHECK(cudaFree(pfe_buffers_[7]));
  GPU_CHECK(cudaFree(pfe_buffers_[8]));

  GPU_CHECK(cudaFree(rpn_buffers_[0]));
  GPU_CHECK(cudaFree(rpn_buffers_[1]));
  GPU_CHECK(cudaFree(rpn_buffers_[2]));
  GPU_CHECK(cudaFree(rpn_buffers_[3]));

  GPU_CHECK(cudaFree(dev_scattered_feature_));

  GPU_CHECK(cudaFree(dev_anchors_px_));
  GPU_CHECK(cudaFree(dev_anchors_py_));
  GPU_CHECK(cudaFree(dev_anchors_pz_));
  GPU_CHECK(cudaFree(dev_anchors_dx_));
  GPU_CHECK(cudaFree(dev_anchors_dy_));
  GPU_CHECK(cudaFree(dev_anchors_dz_));
  GPU_CHECK(cudaFree(dev_anchors_ro_));
  GPU_CHECK(cudaFree(dev_filtered_box_));
  GPU_CHECK(cudaFree(dev_filtered_score_));
  GPU_CHECK(cudaFree(dev_filtered_label_));
  GPU_CHECK(cudaFree(dev_filtered_dir_));
  GPU_CHECK(cudaFree(dev_box_for_nms_));
  GPU_CHECK(cudaFree(dev_filter_count_));

  pfe_context_->destroy();
  rpn_context_->destroy();

  pfe_runtime_->destroy();
  rpn_runtime_->destroy();
  pfe_engine_->destroy();
  rpn_engine_->destroy();
}

void PointPillars::DeviceMemoryMalloc() {
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_x_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_y_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_z_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_i_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_count_histo_),
                       kGridYSize * kGridXSize * sizeof(int)));

  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_x_coors_),
                       kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_y_coors_),
                       kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_num_points_per_pillar_),
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sparse_pillar_map_),
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_pillar_x_),
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_pillar_y_),
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_pillar_z_),
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_pillar_i_),
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));

  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_x_coors_for_sub_shaped_),
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_y_coors_for_sub_shaped_),
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_pillar_feature_mask_),
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));

  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_cumsum_along_x_),
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_cumsum_along_y_),
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  // for make anchor mask kernel
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_min_x_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_min_y_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_max_x_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_max_y_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchor_mask_),
                       kNumAnchor * sizeof(int)));

  // for trt inference
  // create GPU buffers and a stream
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[0],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[1],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[2],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[3],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[4],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[5],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[6],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &pfe_buffers_[7],
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(&pfe_buffers_[8], kPfeOutputSize * sizeof(float)));

  GPU_CHECK(cudaMalloc(&rpn_buffers_[0], kRpnInputSize * sizeof(float)));
  GPU_CHECK(cudaMalloc(&rpn_buffers_[1], kRpnBoxOutputSize * sizeof(float)));
  GPU_CHECK(cudaMalloc(&rpn_buffers_[2], kRpnClsOutputSize * sizeof(float)));
  GPU_CHECK(cudaMalloc(&rpn_buffers_[3], kRpnDirOutputSize * sizeof(float)));

  // for scatter kernel
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_scattered_feature_),
                 kNumThreads * kGridYSize * kGridXSize * sizeof(float)));

  // for filter
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_px_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_py_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_pz_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_dx_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_dy_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_dz_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_ro_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_box_),
                       kNumAnchor * kNumOutputBoxFeature * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_score_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_label_),
                       kNumAnchor * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_dir_),
                       kNumAnchor * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_for_nms_),
                       kNumAnchor * kNumBoxCorners * sizeof(float)));
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_filter_count_), sizeof(int)));
}

void PointPillars::InitAnchors() {
  // allocate memory for anchors
  anchors_px_ = new float[kNumAnchor];
  anchors_py_ = new float[kNumAnchor];
  anchors_pz_ = new float[kNumAnchor];
  anchors_dx_ = new float[kNumAnchor];
  anchors_dy_ = new float[kNumAnchor];
  anchors_dz_ = new float[kNumAnchor];
  anchors_ro_ = new float[kNumAnchor];
  box_anchors_min_x_ = new float[kNumAnchor];
  box_anchors_min_y_ = new float[kNumAnchor];
  box_anchors_max_x_ = new float[kNumAnchor];
  box_anchors_max_y_ = new float[kNumAnchor];
  // deallocate these memories in deconstructor

  GenerateAnchors(anchors_px_, anchors_py_, anchors_pz_, anchors_dx_,
                  anchors_dy_, anchors_dz_, anchors_ro_);

  ConvertAnchors2BoxAnchors(anchors_px_, anchors_py_, anchors_dx_, anchors_dy_,
                            box_anchors_min_x_, box_anchors_min_y_,
                            box_anchors_max_x_, box_anchors_max_y_);

  PutAnchorsInDeviceMemory();
}

void PointPillars::GenerateAnchors(float* anchors_px_, float* anchors_py_,
                                   float* anchors_pz_, float* anchors_dx_,
                                   float* anchors_dy_, float* anchors_dz_,
                                   float* anchors_ro_) {
  // zero clear
  for (int i = 0; i < kNumAnchor; i++) {
    anchors_px_[i] = 0;
    anchors_py_[i] = 0;
    anchors_pz_[i] = 0;
    anchors_dx_[i] = 0;
    anchors_dy_[i] = 0;
    anchors_dz_[i] = 0;
    anchors_ro_[i] = 0;
    box_anchors_min_x_[i] = 0;
    box_anchors_min_y_[i] = 0;
    box_anchors_max_x_[i] = 0;
    box_anchors_max_y_[i] = 0;
  }

  float x_stride = kPillarXSize * 2.0f;
  float y_stride = kPillarYSize * 2.0f;
  float x_offset = kMinXRange + kPillarXSize;
  float y_offset = kMinYRange + kPillarYSize;

  float anchor_x_count[kNumAnchorXInds];
  anchor_x_count[0] = 0;
  for (int i = 0; i < kNumAnchorXInds; i++) {
    anchor_x_count[i] = static_cast<float>(i) * x_stride + x_offset;
  }
  float anchor_y_count[kNumAnchorYInds];
  anchor_y_count[0] = 0;
  for (int i = 0; i < kNumAnchorYInds; i++) {
    anchor_y_count[i] = static_cast<float>(i) * y_stride + y_offset;
  }

  float anchor_r_count[kNumAnchorRInds];
  anchor_r_count[0] = 0;
  anchor_r_count[1] = M_PI / 2;

  // np.meshgrid
  for (int y = 0; y < kNumAnchorYInds; y++) {
    for (int x = 0; x < kNumAnchorXInds; x++) {
      for (int r = 0; r < kNumAnchorRInds; r++) {
        int ind = y * kNumAnchorXInds * kNumAnchorRInds +
                  x * kNumAnchorRInds + r;
        anchors_px_[ind] = anchor_x_count[x];
        anchors_py_[ind] = anchor_y_count[y];
        anchors_ro_[ind] = anchor_r_count[r];
        anchors_pz_[ind] = -1 * kSensorHeight;
        anchors_dx_[ind] = kAnchorDxSize;
        anchors_dy_[ind] = kAnchorDySize;
        anchors_dz_[ind] = kAnchorDzSize;
      }
    }
  }
}

void PointPillars::PutAnchorsInDeviceMemory() {
  GPU_CHECK(cudaMemcpy(dev_box_anchors_min_x_, box_anchors_min_x_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_box_anchors_min_y_, box_anchors_min_y_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_box_anchors_max_x_, box_anchors_max_x_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_box_anchors_max_y_, box_anchors_max_y_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));

  GPU_CHECK(cudaMemcpy(dev_anchors_px_, anchors_px_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_py_, anchors_py_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_pz_, anchors_pz_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_dx_, anchors_dx_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_dy_, anchors_dy_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_dz_, anchors_dz_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_ro_, anchors_ro_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
}

void PointPillars::ConvertAnchors2BoxAnchors(
    float* anchors_px, float* anchors_py, float* anchors_dx, float* anchors_dy,
    float* box_anchors_min_x_, float* box_anchors_min_y_,
    float* box_anchors_max_x_, float* box_anchors_max_y_) {
  // flipping box's dimension
  float flipped_anchors_dx[kNumAnchor];
  flipped_anchors_dx[0] = 0;
  float flipped_anchors_dy[kNumAnchor];
  flipped_anchors_dy[0] = 0;
  for (int x = 0; x < kNumAnchorXInds; x++) {
    for (int y = 0; y < kNumAnchorYInds; y++) {
      int base_ind =
          x * kNumAnchorYInds * kNumAnchorRInds + y * kNumAnchorRInds;
      flipped_anchors_dx[base_ind + 0] = kAnchorDxSize;
      flipped_anchors_dy[base_ind + 0] = kAnchorDySize;
      flipped_anchors_dx[base_ind + 1] = kAnchorDySize;
      flipped_anchors_dy[base_ind + 1] = kAnchorDxSize;
    }
  }
  for (int x = 0; x < kNumAnchorXInds; x++) {
    for (int y = 0; y < kNumAnchorYInds; y++) {
      for (int r = 0; r < kNumAnchorRInds; r++) {
        int ind = x * kNumAnchorYInds * kNumAnchorRInds +
                  y * kNumAnchorRInds + r;
        box_anchors_min_x_[ind] =
            anchors_px[ind] - flipped_anchors_dx[ind] / 2.0f;
        box_anchors_min_y_[ind] =
            anchors_py[ind] - flipped_anchors_dy[ind] / 2.0f;
        box_anchors_max_x_[ind] =
            anchors_px[ind] + flipped_anchors_dx[ind] / 2.0f;
        box_anchors_max_y_[ind] =
            anchors_py[ind] + flipped_anchors_dy[ind] / 2.0f;
      }
    }
  }
}

void PointPillars::InitTRT() {
  // create a TensorRT model from the onnx model and serialize it to a stream
  nvinfer1::IHostMemory* pfe_trt_model_stream{nullptr};
  nvinfer1::IHostMemory* rpn_trt_model_stream{nullptr};
  OnnxToTRTModel(kPfeOnnxFile, &pfe_trt_model_stream);
  OnnxToTRTModel(kRpnOnnxFile, &rpn_trt_model_stream);
  if (pfe_trt_model_stream == nullptr ||
      rpn_trt_model_stream ==
          nullptr) {  // use std:cerr instead of ROS_ERROR because want to keep
                      // this fille ros-agnostics
    std::cerr << "Failed to load ONNX file " << std::endl;
  }

  // deserialize the engine
  pfe_runtime_ = nvinfer1::createInferRuntime(g_logger_);
  rpn_runtime_ = nvinfer1::createInferRuntime(g_logger_);
  if (pfe_runtime_ == nullptr || rpn_runtime_ == nullptr) {
    std::cerr << "Failed to create TensorRT Runtime object." << std::endl;
  }
  pfe_engine_ = pfe_runtime_->deserializeCudaEngine(
      pfe_trt_model_stream->data(), pfe_trt_model_stream->size(), nullptr);
  rpn_engine_ = rpn_runtime_->deserializeCudaEngine(
      rpn_trt_model_stream->data(), rpn_trt_model_stream->size(), nullptr);
  if (pfe_engine_ == nullptr || rpn_engine_ == nullptr) {
    std::cerr << "Failed to create TensorRT Engine." << std::endl;
  }
  pfe_trt_model_stream->destroy();
  rpn_trt_model_stream->destroy();
  pfe_context_ = pfe_engine_->createExecutionContext();
  rpn_context_ = rpn_engine_->createExecutionContext();
  if (pfe_context_ == nullptr || rpn_context_ == nullptr) {
    std::cerr << "Failed to create TensorRT Execution Context." << std::endl;
  }
}

void PointPillars::OnnxToTRTModel(
    const std::string& model_file,  // name of the onnx model
    nvinfer1::IHostMemory**
        trt_model_stream) {  // output buffer for the TensorRT model
  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);

  // create the builder
  nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(g_logger_);
  nvinfer1::INetworkDefinition* network = builder->createNetwork();

  auto parser = nvonnxparser::createParser(*network, g_logger_);

  if (!parser->parseFromFile(model_file.c_str(), verbosity)) {
    std::string msg("failed to parse onnx file");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  // Build the engine
  builder->setMaxBatchSize(kBatchSize);
  builder->setMaxWorkspaceSize(1 << 20);

  nvinfer1::ICudaEngine* engine = builder->buildCudaEngine(*network);

  parser->destroy();

  // serialize the engine, then close everything down
  *trt_model_stream = engine->serialize();
  engine->destroy();
  network->destroy();
  builder->destroy();
}

void PointPillars::PreprocessCPU(const float* in_points_array,
                                 const int in_num_points) {
  int x_coors[kMaxNumPillars];
  x_coors[0] = 0;
  int y_coors[kMaxNumPillars];
  y_coors[0] = 0;
  float num_points_per_pillar[kMaxNumPillars];
  num_points_per_pillar[0] = 0;
  float* pillar_x = new float[kMaxNumPillars * kMaxNumPointsPerPillar];
  float* pillar_y = new float[kMaxNumPillars * kMaxNumPointsPerPillar];
  float* pillar_z = new float[kMaxNumPillars * kMaxNumPointsPerPillar];
  float* pillar_i = new float[kMaxNumPillars * kMaxNumPointsPerPillar];

  float* x_coors_for_sub_shaped =
      new float[kMaxNumPillars * kMaxNumPointsPerPillar];
  float* y_coors_for_sub_shaped =
      new float[kMaxNumPillars * kMaxNumPointsPerPillar];
  float* pillar_feature_mask =
      new float[kMaxNumPillars * kMaxNumPointsPerPillar];

  float* sparse_pillar_map = new float[kNumIndsForScan * kNumIndsForScan];

  preprocess_points_ptr_->Preprocess(
      in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar,
      pillar_x, pillar_y, pillar_z, pillar_i, x_coors_for_sub_shaped,
      y_coors_for_sub_shaped, pillar_feature_mask, sparse_pillar_map,
      host_pillar_count_);

  GPU_CHECK(cudaMemset(dev_x_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(
      dev_pillar_x_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_pillar_y_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_pillar_z_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_pillar_i_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_x_coors_for_sub_shaped_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_y_coors_for_sub_shaped_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0,
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0,
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  GPU_CHECK(cudaMemcpy(dev_x_coors_, x_coors, kMaxNumPillars * sizeof(int),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_y_coors_, y_coors, kMaxNumPillars * sizeof(int),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(
      cudaMemcpy(dev_pillar_x_, pillar_x,
                 kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
                 cudaMemcpyHostToDevice));
  GPU_CHECK(
      cudaMemcpy(dev_pillar_y_, pillar_y,
                 kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
                 cudaMemcpyHostToDevice));
  GPU_CHECK(
      cudaMemcpy(dev_pillar_z_, pillar_z,
                 kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
                 cudaMemcpyHostToDevice));
  GPU_CHECK(
      cudaMemcpy(dev_pillar_i_, pillar_i,
                 kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
                 cudaMemcpyHostToDevice));
  GPU_CHECK(
      cudaMemcpy(dev_x_coors_for_sub_shaped_, x_coors_for_sub_shaped,
                 kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
                 cudaMemcpyHostToDevice));
  GPU_CHECK(
      cudaMemcpy(dev_y_coors_for_sub_shaped_, y_coors_for_sub_shaped,
                 kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
                 cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_num_points_per_pillar_, num_points_per_pillar,
                       kMaxNumPillars * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(
      cudaMemcpy(dev_pillar_feature_mask_, pillar_feature_mask,
                 kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
                 cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_sparse_pillar_map_, sparse_pillar_map,
                       kNumIndsForScan * kNumIndsForScan * sizeof(float),
                       cudaMemcpyHostToDevice));

  delete[] pillar_x;
  delete[] pillar_y;
  delete[] pillar_z;
  delete[] pillar_i;
  delete[] x_coors_for_sub_shaped;
  delete[] y_coors_for_sub_shaped;
  delete[] pillar_feature_mask;
  delete[] sparse_pillar_map;
}

void PointPillars::PreprocessGPU(const float* in_points_array,
                                 const int in_num_points) {
  float* dev_points;
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_points),
                       in_num_points * kNumBoxCorners * sizeof(float)));
  GPU_CHECK(cudaMemcpy(dev_points, in_points_array,
                       in_num_points * kNumBoxCorners * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemset(dev_pillar_count_histo_, 0,
                       kGridYSize * kGridXSize * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0,
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));
  GPU_CHECK(cudaMemset(
      dev_pillar_x_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_pillar_y_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_pillar_z_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(
      dev_pillar_i_, 0,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_x_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0,
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_anchor_mask_, 0, kNumAnchor * sizeof(int)));

  preprocess_points_cuda_ptr_->DoPreprocessPointsCuda(
      dev_points, in_num_points, dev_x_coors_, dev_y_coors_,
      dev_num_points_per_pillar_, dev_pillar_x_, dev_pillar_y_, dev_pillar_z_,
      dev_pillar_i_, dev_x_coors_for_sub_shaped_, dev_y_coors_for_sub_shaped_,
      dev_pillar_feature_mask_, dev_sparse_pillar_map_, host_pillar_count_);

  GPU_CHECK(cudaFree(dev_points));
}

void PointPillars::Preprocess(const float* in_points_array,
                              const int in_num_points) {
  if (kReproduceResultMode) {
    PreprocessCPU(in_points_array, in_num_points);
  } else {
    PreprocessGPU(in_points_array, in_num_points);
  }
}

void PointPillars::DoInference(const float* in_points_array,
                               const int in_num_points,
                               std::vector<float>* out_detections,
                               std::vector<int>* out_labels) {
  Preprocess(in_points_array, in_num_points);

  anchor_mask_cuda_ptr_->DoAnchorMaskCuda(
      dev_sparse_pillar_map_, dev_cumsum_along_x_, dev_cumsum_along_y_,
      dev_box_anchors_min_x_, dev_box_anchors_min_y_, dev_box_anchors_max_x_,
      dev_box_anchors_max_y_, dev_anchor_mask_);

  cudaStream_t stream;
  GPU_CHECK(cudaStreamCreate(&stream));
  GPU_CHECK(cudaMemcpyAsync(
      pfe_buffers_[0], dev_pillar_x_,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
      cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(
      pfe_buffers_[1], dev_pillar_y_,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
      cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(
      pfe_buffers_[2], dev_pillar_z_,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
      cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(
      pfe_buffers_[3], dev_pillar_i_,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
      cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[4], dev_num_points_per_pillar_,
                            kMaxNumPillars * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(
      pfe_buffers_[5], dev_x_coors_for_sub_shaped_,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
      cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(
      pfe_buffers_[6], dev_y_coors_for_sub_shaped_,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
      cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(
      pfe_buffers_[7], dev_pillar_feature_mask_,
      kMaxNumPillars * kMaxNumPointsPerPillar * sizeof(float),
      cudaMemcpyDeviceToDevice, stream));
  pfe_context_->enqueue(kBatchSize, pfe_buffers_, stream, nullptr);

  GPU_CHECK(
      cudaMemset(dev_scattered_feature_, 0, kRpnInputSize * sizeof(float)));
  scatter_cuda_ptr_->DoScatterCuda(
      host_pillar_count_[0], dev_x_coors_, dev_y_coors_,
      reinterpret_cast<float*>(pfe_buffers_[8]), dev_scattered_feature_);

  GPU_CHECK(cudaMemcpyAsync(rpn_buffers_[0], dev_scattered_feature_,
                            kBatchSize * kRpnInputSize * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  rpn_context_->enqueue(kBatchSize, rpn_buffers_, stream, nullptr);

  GPU_CHECK(cudaMemset(dev_filter_count_, 0, sizeof(int)));
  postprocess_cuda_ptr_->DoPostprocessCuda(
      reinterpret_cast<float*>(rpn_buffers_[1]),
      reinterpret_cast<float*>(rpn_buffers_[2]),
      reinterpret_cast<float*>(rpn_buffers_[3]), dev_anchor_mask_,
      dev_anchors_px_, dev_anchors_py_, dev_anchors_pz_, dev_anchors_dx_,
      dev_anchors_dy_, dev_anchors_dz_, dev_anchors_ro_, dev_filtered_box_,
      dev_filtered_score_, dev_filtered_label_, dev_filtered_dir_,
      dev_box_for_nms_, dev_filter_count_, out_detections, out_labels);

  // release the stream and the buffers
  cudaStreamDestroy(stream);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
