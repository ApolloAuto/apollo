# 如何添加新的lidar匹配算法

Perception中的lidar数据流如下：
![](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/images/lidar_perception_data_flow.png)

本篇文档所介绍的lidar检测算法位于图中的Recognition Component中。当前Recognition Component的架构如下：
![lidar recognition](images/lidar_recognition.png)

从以上结构中可以清楚地看到lidar匹配算法是位于Recognition Component的 `base_lidar_obstacle_tracking` 中的抽象成员类 `base_multi_target_tracker` 的派生类。下面将详细介绍如何基于当前结构添加新的lidar匹配算法。

Apollo默认的lidar匹配算法为MlfEngine，它可以轻松更改或替换为不同的算法。本篇文档将介绍如何引入新的lidar匹配算法，添加新算法的步骤如下：

1. 定义一个继承基类 `base_multi_target_tracker` 的类
2. 实现新类 `NewLidarTracker`
3. 为新类 `NewLidarTracker` 配置config的proto文件
4. 更新 lidar_obstacle_tracking.conf

为了更好的理解，下面对每个步骤进行详细的阐述:

## 定义一个继承基类 `base_multi_target_tracker` 的类

所有的lidar匹配算法都必须继承基类 `base_multi_target_tracker`，它定义了一组接口。 以下是匹配算法继承基类的示例:

```c++
namespace apollo {
namespace perception {
namespace lidar {

class NewLidarTracker : public BaseMultiTargetTracker {
 public:
  NewLidarTracker();
  virtual ~NewLidarTracker() = default;

  bool Init(const MultiTargetTrackerInitOptions& options =
                        MultiTargetTrackerInitOptions()) override;

  bool Track(const MultiTargetTrackerOptions& options, LidarFrame* frame) override;

  std::string Name() const override;

};  // class NewLidarTracker

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```

基类 `base_multi_target_tracker` 已定义好各虚函数签名，接口信息如下：

```c++
struct MultiTargetTrackerInitOptions {};

struct MultiTargetTrackerOptions {};

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

## 实现新类 `NewLidarTracker`

为了确保新的匹配算法能顺利工作，`NewLidarTracker`至少需要重写`base_multi_target_tracker`中定义的接口Init(),Track()和Name()。其中Init()函数负责完成加载配置文件，初始化类成员等工作；而Track()则负责实现算法的主体流程。一个具体的`NewLidarTracker.cc`实现示例如下：

```c++
namespace apollo {
namespace perception {
namespace lidar {

bool NewLidarTracker::Init(const MultiTargetTrackerInitOptions& options) {
    /*
    你的算法初始化部分
    */
}

bool NewLidarTracker::Track(const MultiTargetTrackerOptions& options, LidarFrame* frame) {
    /*
    你的算法实现部分
    */
}

std::string NewLidarTracker::Name() const {
    /*
    返回你的匹配算法名称
    */
}

PERCEPTION_REGISTER_MULTITARGET_TRACKER(NewLidarTracker); //注册新的lidar_tracker

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
```


## 为新类 `NewLidarTracker` 配置config的proto文件

按照下面的步骤添加新lidar匹配算法的配置和参数信息:

1. 根据算法要求为新lidar匹配算法配置config的`proto`文件。作为示例，可以参考以下位置的`multi_lidar_fusion`的`proto`定义：`modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fustion_config.proto`

2. 定义新的`proto`之后，例如`newlidartracker_config.proto`，输入以下内容:

    ```protobuf
    syntax = "proto2";
    package apollo.perception.lidar;

    message NewLidarTrackerConfig {
        double parameter1 = 1;
        int32 parameter2 = 2;
    }
    ```

3. 参考如下内容更新 `modules/perception/production/conf/perception/lidar/config_manager.config`文件:

    ```protobuf
    model_config_path: "./conf/perception/lidar/modules/newlidartracker_config.config"
    ```

4. 参考同级别目录下 `modules/multi_lidar_fusion.config` 内容创建 `newlidartracker.config`:

    ```protobuf
    model_configs {
    name: "NewLidarTracker"
        version: "1.0.0"
        string_params {
            name: "root_path"
            value: "./data/perception/lidar/models/newlidartracker"
        }
    }
    ```

5. 参考 `multi_lidar_tracker` 在目录 `modules/perception/production/data/perception/lidar/models/` 中创建 `newlidartracker` 文件夹，并根据需求创建不同传感器的 `.conf` 文件：

    ```
    注意：此处 "*.conf" 文件应对应步骤1，2中的proto文件格式.
    ```

## 更新 lidar_obstacle_tracking.conf

要使用Apollo系统中的新lidar匹配算法，需要将 `modules/perception/production/data/perception/lidar/models/lidar_obstacle_pipline` 中的对应传感器的 `lidar_obstacle_tracking.conf` 文件的 `multi_target_tracker` 字段值改为 "NewLidarTracker"

在完成以上步骤后，您的新lidar匹配算法便可在Apollo系统中生效。
