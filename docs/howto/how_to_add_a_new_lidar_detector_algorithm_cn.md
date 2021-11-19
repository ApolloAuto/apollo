# 如何添加新的lidar检测算法

Perception中的lidar数据流如下：
![](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/images/lidar_perception_data_flow.png)

本篇文档所介绍的lidar检测算法位于图中的Detection Component中。当前Detection Component的架构如下：
![lidar detection high-level](images/lidar_detection_1.png)
![lidar detection](images/lidar_detection_2.png)

从以上结构中可以清楚地看到lidar检测算法是位于Detection Component的 `base_lidar_obstacle_detection` 中的抽象成员类 `base_lidar_detector` 的派生类。下面将详细介绍如何基于当前结构添加新的lidar检测算法。

Apollo默认提供了2种lidar检测算法--PointPillars和CNN（NCut不再维护），可以轻松更改或替换为不同的算法。每种算法的输入都是原始点云信息，输出都是目标级障碍物信息。本篇文档将介绍如何引入新的lidar检测算法，添加新算法的步骤如下：

1. 定义一个继承基类 `base_lidar_detector` 的类
2. 实现新类 `NewLidarDetector`
3. 为新类 `NewLidarDetector` 配置config和param的proto文件
4. 更新 lidar_obstacle_detection.conf

为了更好的理解，下面对每个步骤进行详细的阐述:

## 定义一个继承基类 `base_lidar_detector` 的类

所有的lidar检测算法都必须继承基类`base_lidar_detector`，它定义了一组接口。 以下是检测算法继承基类的示例:

```c++
namespace apollo {
namespace perception {
namespace lidar {

class NewLidarDetector : public BaseLidarDetector {
 public:
  NewLidarDetector();
  virtual ~NewLidarDetector() = default;

  bool Init(const LidarDetectorInitOptions& options = LidarDetectorInitOptions()) override;

  bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;

  std::string Name() const override;

};  // class NewLidarDetector

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

基类 `base_lidar_detector` 已定义好各虚函数签名，接口信息如下：

```c++
struct LidarDetectorInitOptions {
  std::string sensor_name = "velodyne64";
};

struct LidarDetectorOptions {};

struct LidarFrame {
  // point cloud
  std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud;
  // world point cloud
  std::shared_ptr<base::AttributePointCloud<base::PointD>> world_cloud;
  // timestamp
  double timestamp = 0.0;
  // lidar to world pose
  Eigen::Affine3d lidar2world_pose = Eigen::Affine3d::Identity();
  // lidar to world pose
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
  // hdmap struct
  std::shared_ptr<base::HdmapStruct> hdmap_struct = nullptr;
  // segmented objects
  std::vector<std::shared_ptr<base::Object>> segmented_objects;
  // tracked objects
  std::vector<std::shared_ptr<base::Object>> tracked_objects;
  // point cloud roi indices
  base::PointIndices roi_indices;
  // point cloud non ground indices
  base::PointIndices non_ground_indices;
  // secondary segmentor indices
  base::PointIndices secondary_indices;
  // sensor info
  base::SensorInfo sensor_info;
  // reserve string
  std::string reserve;

  void Reset();

  void FilterPointCloud(base::PointCloud<base::PointF> *filtered_cloud,
                        const std::vector<uint32_t> &indices);
};
```

## 实现新类 `NewLidarDetector`

为了确保新的检测算法能顺利工作，`NewLidarDetector`至少需要重写`base_lidar_detector`中定义的接口Init(),Detect()和Name()。其中Init()函数负责完成加载配置文件，初始化类成员等工作；而Detect()则负责实现算法的主体流程。一个具体的`NewLidarDetector.cc`实现示例如下：

```c++
namespace apollo {
namespace perception {
namespace lidar {

bool NewLidarDetector::Init(const LidarDetectorInitOptions& options) {
    /*
    你的算法初始化部分
    */
}

bool NewLidarDetector::Detect(const LidarDetectorOptions& options, LidarFrame* frame) {
    /*
    你的算法实现部分
    */
}

std::string NewLidarDetector::Name() const {
    /*
    返回你的检测算法名称
    */
}

PERCEPTION_REGISTER_LIDARDETECTOR(NewLidarDetector); //注册新的lidar_detector

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```


## 为新类 `NewLidarDetector` 配置config和param的proto文件

按照下面的步骤添加新lidar检测算法的配置和参数信息:

1. 根据算法要求为新lidar检测算法配置config的`proto`文件。作为示例，可以参考以下位置的`cnn_segmentation`的`proto`定义：`modules/perception/lidar/lib/detector/cnn_segmentation/proto/cnnseg_config.proto`

2. 定义新的`proto`之后，例如`newlidardetector_config.proto`，输入以下内容:

    ```protobuf
    syntax = "proto2";
    package apollo.perception.lidar;

    message NewLidarDetectorConfig {
        double parameter1 = 1;
        int32 parameter2 = 2;
    }
    ```
3. 根据算法要求为新lidar检测算法配置param的`proto`文件。作为示例，可以参考以下位置的`cnn_segmentation`的`proto`定义：`modules/perception/lidar/lib/detector/cnn_segmentation/proto/cnnseg_param.proto`。同样地，在定义完成后输入以下内容：

    ```protobuf
    syntax = "proto2";
    package apollo.perception.lidar;

    //你的param参数
    ```

4. 参考如下内容更新 `modules/perception/production/conf/perception/lidar/config_manager.config`文件:

    ```protobuf
    model_config_path: "./conf/perception/lidar/modules/newlidardetector_config.config"
    ```

5. 参考同级别目录下 `modules/cnnseg.config` 内容创建 `newlidardetector.config`:

    ```protobuf
    model_configs {
        name: "NewLidarDetector"
        version: "1.0.0"
        string_params {
            name: "root_path"
            value: "./data/perception/lidar/models/newlidardetector"
        }
    }
    ```

6. 参考 `cnnseg` 在目录 `modules/perception/production/data/perception/lidar/models/` 中创建 `newlidardetector` 文件夹，并根据需求创建不同传感器的 `.conf` 文件：

    ```
    注意：此处 "*.conf" 和 "*param.conf" 文件应对应步骤1，2，3中的proto文件格式.
    ```

## 更新 lidar_obstacle_detection.conf

要使用Apollo系统中的新lidar检测算法，需要将 `modules/perception/production/data/perception/lidar/models/lidar_obstacle_pipline` 中的对应传感器的 `lidar_obstacle_detection.conf` 文件的 `detector` 字段值改为 "NewLidarDetector"

在完成以上步骤后，您的新lidar检测算法便可在Apollo系统中生效。
