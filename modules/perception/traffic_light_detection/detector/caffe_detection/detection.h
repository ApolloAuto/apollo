/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

/*
功能	方法
初始化	Init()
检测交通灯	Detect()
运行神经网络	Inference()
筛选交通灯框	SelectOutputBoxes()
去除重叠框	ApplyNMS()
*/

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/traffic_light_detection/detector/caffe_detection/proto/model_param.pb.h"

#include "cyber/common/macros.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/image_8u.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/traffic_light_detection/algorithm/cropbox.h"
#include "modules/perception/traffic_light_detection/algorithm/select.h"
#include "modules/perception/traffic_light_detection/interface/base_traffic_light_detector.h"

namespace apollo {
namespace perception {
namespace trafficlight {

class TrafficLightDetection : public BaseTrafficLightDetector {
 public:
  /**
   * @brief Construct a new traffic light detection object.
   * 
   */
  TrafficLightDetection();
  /**
   * @brief Destroy the traffic light detection object.
   * 
   */
  ~TrafficLightDetection() = default;
  /**
   * @brief Initialize traffic light detector parameters.
   * 
   * @param options 
   * @return true 
   * @return false 
   */
  // 初始化交通灯检测器，加载模型参数
  // 初始化交通灯检测器
  // 可能涉及 加载模型参数、设置推理器、初始化 GPU 资源等
  bool Init(const TrafficLightDetectorInitOptions &options) override;

  /**
   * @brief Detect traffic light from image.
   * 
   * @param frame 
   * @return true 
   * @return false 
   */
  // 在图像中检测交通信号灯
  // 从输入图像中检测交通信号灯
  // 主要调用：
  // 1.Inference()：使用深度学习模型进行交通灯检测
  // 2.SelectOutputBoxes()：筛选有效检测结果
  // 3.ApplyNMS()：去除重叠的框
  bool Detect(camera::TrafficLightFrame *frame) override;
  /**
   * @brief Dump output of inference results.
   * 
   * @param crop_box_list 
   * @param resize_scale_list_col 
   * @param resize_scale_list_row 
   * @param lights 
   * @return true 
   * @return false 
   */
  // 对推理结果进行后处理，筛选有效的交通灯框
  // 对模型输出的检测框进行筛选和转换
  // 考虑缩放比例，将推理结果映射回原始图像坐标
  bool SelectOutputBoxes(const std::vector<base::RectI> &crop_box_list,
                         const std::vector<float> &resize_scale_list_col,
                         const std::vector<float> &resize_scale_list_row,
                         std::vector<base::TrafficLightPtr> *lights);
  /**
   * @brief Filter overlapping boxes using NMS.
   * 
   * @param lights 
   * @param iou_thresh 
   */
  // 使用非极大值抑制（NMS）过滤重叠的检测框
  // 使用非极大值抑制（NMS），过滤重叠的检测框，保留最优结果
  // iou_thresh（交并比阈值）用于判定两个框是否重叠
  void ApplyNMS(std::vector<base::TrafficLightPtr> *lights,
                double iou_thresh = 0.6);
  /**
   * @brief Model inference process.
   * 
   * @param lights 
   * @param data_provider 
   * @return true 
   * @return false 
   */
  // 执行神经网络推理（深度学习模型）
  // 运行神经网络模型，进行推理（Inference）
  // 使用 rt_net_ 执行深度学习计算，预测交通灯的位置
  bool Inference(std::vector<base::TrafficLightPtr> *lights,
                 camera::DataProvider *data_provider);
  /**
   * @brief Get the detected boxes object.
   * 
   * @return const std::vector<base::TrafficLightPtr>& 
   */
  const std::vector<base::TrafficLightPtr> &getDetectedBoxes() {
    return detected_bboxes_;
  }

 private:
  trafficlight::ModelParam detection_param_;  // 存储交通灯检测模型的参数
  std::string detection_root_dir;             // 存储模型文件路径

  camera::DataProvider::ImageOptions data_provider_image_option_;   // 图像预处理参数
  // rt_net_ 是神经网络推理器，用来执行深度学习推理，可能基于 TensorRT、Caffe、ONNX Runtime 等
  std::shared_ptr<inference::Inference> rt_net_ = nullptr;          // 神经网络推理器，执行深度学习推理
  std::shared_ptr<base::Image8U> image_ = nullptr;                  // 处理后的图像
  std::shared_ptr<base::Blob<float>> param_blob_;                   // 神经网络的输入参数
  std::shared_ptr<base::Blob<float>> mean_buffer_;                  // 归一化的均值
  std::shared_ptr<IGetBox> crop_;                                   // 裁剪算法，用于从原始图像裁剪交通灯区域
  // detected_bboxes_：存储所有检测到的交通灯（可能包含误检）
  std::vector<base::TrafficLightPtr> detected_bboxes_;              // 存储所有检测到的交通灯框
  // selected_bboxes_：经过筛选后的交通灯
  std::vector<base::TrafficLightPtr> selected_bboxes_;              // 存储筛选后的交通灯框
  std::vector<std::string> net_inputs_;                  
  std::vector<std::string> net_outputs_;
  Select select_;    // 选择有效的交通灯
  int max_batch_size_;
  int param_blob_length_;
  float mean_[3];
  std::vector<base::RectI> crop_box_list_;
  std::vector<float> resize_scale_list_;
  int gpu_id_;      // 指定 GPU 设备 ID

  DISALLOW_COPY_AND_ASSIGN(TrafficLightDetection);
};  // class TrafficLightDetection

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
