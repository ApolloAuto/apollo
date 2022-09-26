## Apollo与Paddle3D
- 在感知模块中，Apollo提供了众多的感知模型以满足开发者的需求，如基于Camera Detection的Smoke模型，基于Lidar Detection的PointPillars、MaskPillars等。我们鼓励开发者在Apollo平台上贡献或适配引入新的感知模型并进行迭代，为此我们与PaddlePaddle团队合作，在Apollo感知框架中添加了对Paddle Inference推理引擎的支持，并引入了Paddle3D，开发者可以方便的使用Paddle3D进行模型的开发和迭代，并方便的将感知模型移植到Apollo感知框架。
## Paddle3D简介
- Paddle3D是飞桨官方开源的端到端深度学习3D感知套件，涵盖了许多前沿和经典的3D感知模型，支持多种模态和多种任务，可以助力开发者便捷地完成『自动驾驶』领域模型从训练到部署的全流程应用。Paddle3D的详细使用请参考[Paddle3D官网](https://github.com/PaddlePaddle/Paddle3D)。

## Apollo 当前支持的Paddle3D模型列表
|模型名称 | 模型类型 | 支持形式 |  模型表现|
|:---|:---|:---|:---|
|CenterPoint  |  Lidar Detection |  Apollo内集成  | [CenterPoint模型指标](https://github.com/PaddlePaddle/Paddle3D/tree/develop/docs/models/centerpoint)   |
|Caddn |  Camera Detection |  Apollo内集成 |    |
|Smoke(Paddle3D版本)| Camera Detection  | 需要用户参照文档进行模型适配  |    |
| PointPillars(Paddle3D版本)|  Lidar Detection | 需要用户参照文档进行模型适配 |[PointPillars模型指标](https://github.com/PaddlePaddle/Paddle3D/tree/develop/docs/models/pointpillars)   |

## 如何添加Lidar Detection模型
#### 1.添加模型文件
- 将训练好的模型文件存放于`modules/perception/production/data/perception/lidar/models/detection/xxx`目录下。
#### 2.添加新的 Lidar Detector
- 参考[how_to_add_a_new_lidar_detector_algorithm](../../docs/howto/how_to_add_a_new_lidar_detector_algorithm_cn.md)，创建一个新的Lidar Detector。
#### 3.修改Lidar Detector以适配新的Lidar Detection模型
#### 4.创建并初始化predictor

新添加的Detector中的`init`函数主要用来对predictor进行配置并进行初始化，下面以`CenterPointDetection::Init`为例进行说明，更具体请参考[Paddle Inference C++ API](https://paddle-inference.readthedocs.io/en/master/api_reference/cxx_api_doc/cxx_api_index.html)。

```C++
bool CenterPointDetection::Init(const LidarDetectorInitOptions &options) {
  // 创建默认配置对象 
  paddle::AnalysisConfig config;
  // 启用GPU
  config.EnableUseGpu(1000, FLAGS_gpu_id);
  // 设置预测模型路径
  config.SetModel(FLAGS_center_point_model_file,
                  FLAGS_center_point_params_file);
  // 是否开启TensorRT加速
  if (FLAGS_use_trt) {
    // 精度选择
    paddle::AnalysisConfig::Precision precision;
    if (FLAGS_trt_precision == 0) {
      precision = paddle_infer::PrecisionType::kFloat32;
    } else if (FLAGS_trt_precision == 1) {
      precision = paddle_infer::PrecisionType::kHalf;
    } else {
      AERROR << "Tensorrt type can only support 0 or 1, but recieved is"
             << FLAGS_trt_precision << "\n";
      return false;
    }
    config.EnableTensorRtEngine(1 << 30, 1, 3, precision, FLAGS_trt_use_static,
                                false);
    // 载入动态shape文件
    config.EnableTunedTensorRtDynamicShape(FLAGS_dynamic_shape_file, true);
    // 是否使用反序列化
    if (FLAGS_trt_use_static) {
      config.SetOptimCacheDir(FLAGS_trt_static_dir);
    }
  }
  // 是否开启IR优化
  config.SwitchIrOptim(true);
  // 创建predictor
  predictor_ = paddle_infer::CreatePredictor(config);
  return true;
}
```
#### 5.Detect()函数的处理流程
数据的预处理、推理、后处理、感知结果发布等流程都是在Detect()函数中实现的。获取点云数据的过程可以参考`ceter_point_detector.cc`的`bool CenterPointDetection::Detect()`函数的流程即可。

- Detect()函数包含了点云数据的预处理，前处理、推理、后处理、感知结果发布等流程。其中，点云数据的预处理主要包括`DownSample -> Fuse -> Shuffle`等流程，处理流程相对固定，通过`CloudToArray()`函数，将预处理后的点云存储到了数组中，用户根据自己的模型需要，对存储到数组中的点云数据进行前处理即可。
- 后处理完成后，用户通过`GetObjects()`将目标级结果存储到`frame`中。用户需要确认自己模型输出的3D bounding-box的格式及顺序。

## 如何添加Camera Detection模型

#### 1.添加模型文件
- 将训练好的模型文件存放于`modules/perception/production/data/perception/camera/models/yolo_obstacle_detector/xxx`目录下。

#### 2.添加新的Camera Detector
- 参考[how_to_add_a_new_camera_detector_algorithm](../../docs/howto/how_to_add_a_new_camera_detector_algorithm_cn.md)，创建一个新的Camera Detector。

#### 3.修改Camera Detector以适配新的Camera Detection模型
- 根据模型结构修改对应的proto文件：Camera Detector通过`blob`数据结构来管理和存储模型的输入和输出数据。用户需要根据模型结构，在该模型对应的proto文件中添加和对应的输入输出项，可参考`smoke.proto`文件的`NetworkParam`字段，并添加对应的proto配置文件，参考`smoke-config.pt`。

- 同Lidar Detection类似，Camera Detection中的Detect()函数同样包含了图像数据的预处理，前处理、推理、后处理、感知结果发布等流程。通过`frame->data_provider->GetImage()`函数，用户可以获取图像数据，通过`inference::ResizeGPU()`函数，可以对图像数据进行resize等预处理操作。
- 后处理完成后，用户需要根据模型的特点，将2d bounding-box、3d bounding-box存储到`std::vector<base::ObjectPtr>`结构中。具体可参考`region_output.cc`中的`get_smoke_objects_cpu`函数。
