# 如何添加新的camera匹配算法

Perception中的camera数据流如下：
    ![camera overview](images/Camera_overview.png)

本篇文档所介绍的camera匹配算法分为两种，分别为针对交通信号灯的匹配算法和针对障碍物的匹配算法（针对车道线的匹配算法虽然已预留接口类，但目前暂未部署）。这两种匹配算法分别位于图中的Traffic_light和Obstacle三两大Component中。各Component的架构如下：

交通信号灯感知:
    ![traffic light component](images/camera_traffic_light_detection.png)

障碍物感知:
    ![obstacle component](images/camera_obstacle_detection.png)


从以上结构中可以清楚地看到,各个component都有自己的抽象类成员 `base_XXX_tracker`。对应的匹配算法作为 `base_XXX_tracker` 的不同的派生类，继承各自的基类实现算法的部署。由于各tracker基类在结构上非常相似，下面将以 ` base_obstacle_tracker` 为例介绍如何基于当前结构添加新的camera障碍物匹配算法。新增交通信号灯匹配算法的步骤相同。


Apollo在Obstacle Detection中默认提供了1种camera匹配算法--OMTObstacleTracker，它们可以被轻松更改或替换为不同的算法。算法的输入都是经过检测算法识别的目标级障碍物信息，输出都是经过匹配跟踪算法筛选后的目标级障碍物信息。本篇文档将介绍如何引入新的Camera匹配算法，添加新算法的步骤如下：

1. 定义一个继承基类 `base_obstacle_tracker` 的类
2. 实现新类 `NewObstacleTracker`
3. 为新类 `NewObstacleTracker` 配置param的proto文件
4. 更新config文件使新的算法生效

为了更好的理解，下面对每个步骤进行详细的阐述:

## 定义一个继承基类 `base_obstacle_tracker` 的类

所有的camera匹配算法都必须继承基类`base_obstacle_tracker`，它定义了一组接口。 以下是匹配算法继承基类的示例:

```c++
namespace apollo {
namespace perception {
namespace camera {

class NewObstacleTracker : public BaseObstacleTracker {
 public:
  NewObstacleTracker();
  virtual ~NewObstacleTracker() = default;

  bool Init(const ObstacleTrackerInitOptions& options) override;

  bool Predict(const ObstacleTrackerOptions &options,
               CameraFrame *frame) override;

  bool Associate2D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame) override;

  bool Associate3D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame) override;

  bool Track(const ObstacleTrackerOptions& options,
             CameraFrame* frame) override;

  std::string Name() const override;

};  // class NewObstacleTracker

}  // namespace camera
}  // namespace perception
}  // namespace apollo
```

基类 `base_obstacle_tracker` 已定义好各虚函数签名，接口信息如下：

```c++
struct ObstacleTrackerInitOptions : public BaseInitOptions {
  float image_width;
  float image_height;
};

struct ObstacleTrackerOptions {};

struct CameraFrame {
  // timestamp
  double timestamp = 0.0;
  // frame sequence id
  int frame_id = 0;
  // data provider
  DataProvider *data_provider = nullptr;
  // calibration service
  BaseCalibrationService *calibration_service = nullptr;
  // hdmap struct
  base::HdmapStructPtr hdmap_struct = nullptr;
  // tracker proposed objects
  std::vector<base::ObjectPtr> proposed_objects;
  // segmented objects
  std::vector<base::ObjectPtr> detected_objects;
  // tracked objects
  std::vector<base::ObjectPtr> tracked_objects;
  // feature of all detected object ( num x dim)
  // detect lane mark info
  std::vector<base::LaneLine> lane_objects;
  std::vector<float> pred_vpt;
  std::shared_ptr<base::Blob<float>> track_feature_blob = nullptr;
  std::shared_ptr<base::Blob<float>> lane_detected_blob = nullptr;
  // detected traffic lights
  std::vector<base::TrafficLightPtr> traffic_lights;
  // camera intrinsics
  Eigen::Matrix3f camera_k_matrix = Eigen::Matrix3f::Identity();
  // narrow to obstacle projected_matrix
  Eigen::Matrix3d project_matrix = Eigen::Matrix3d::Identity();
  // camera to world pose
  Eigen::Affine3d camera2world_pose = Eigen::Affine3d::Identity();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // struct CameraFrame
```

## 实现新类 `NewObstacleTracker`

为了确保新的匹配算法能顺利工作，`NewObstacleTracker` 至少需要重写 `base_obstacle_tracker` 中定义的接口Init(),Track()和Name()。其中Init()函数负责完成加载配置文件，初始化类成员等工作；而Track()则负责实现算法的主体流程。一个具体的`NewObstacleTracker.cc`实现示例如下：

```
注意：当前版本base_obstacle_tracker.h尚未将算法流程封装到Track()函数中，需要完全重写其所有接口函数。
```

```c++
namespace apollo {
namespace perception {
namespace camera {

bool NewObstacleTracker::Init(const ObstacleTrackerInitOptions& options) {
    /*
    你的算法初始化部分
    */
}

bool NewObstacleTracker::Track(const ObstacleTrackerInitOptions& options,
                               CameraFrame *frame) {
    /*
    你的算法实现部分
    */
}

bool NewObstacleTracker::Predict(const ObstacleTrackerOptions &options,
                                 CameraFrame *frame) {
    /*
    你的算法实现部分--预测
    */
}

  bool Associate2D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame){
    /*
    你的算法实现部分--2D匹配
    */
}

  bool Associate3D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame){
    /*
    你的算法实现部分--3D匹配
    */
}

std::string NewObstacleTracker::Name() const {
    /*
    返回你的匹配算法名称
    */
}

REGISTER_OBSTACLE_TRACKER(NewObstacleTracker); //注册新的camera_obstacle_tracker

}  // namespace camera
}  // namespace perception
}  // namespace apollo
```


## 为新类 `NewObstacleTracker` 配置param的proto文件

按照下面的步骤添加新camera匹配算法的参数信息:

1. 根据算法要求为新camera匹配算法配置param的`proto`文件。当然，如果参数适配，您也可以直接使用现有的`proto`文件，或者对现有`proto`文件进行更改。作为示例，可以参考以下位置的`omt`的`proto`定义：`modules/perception/camera/lib/obstacle/tracker/omt/proto/omt.proto`。定义完成后在文件头部输入以下内容：

    ```protobuf
    syntax = "proto2";
    package apollo.perception.camera.NewObstacleTracker;

    //你的param参数
    ```

2. 参考 `omt_obstacle_tracker` 在目录 `modules/perception/production/data/perception/camera/models/` 中创建 `new_obstacle_tracker` 文件夹，并根据需求创建 `*.pt` 文件：

    ```
    注意：此处 "*.pt" 文件应对应步骤1中的proto文件格式.
    ```

## 更新config文件使新的算法生效

要使用Apollo系统中的新camera匹配算法，需要根据需求依次对以下config文件进行配置:

1. 参考如下内容更新 `modules/perception/production/conf/perception/camera/obstacle.pt`文件,将之前步骤中新建的 `*.pt` 配置到加载路径中:

    ```protobuf
    tracker_param {
    plugin_param{
        name : "NewObstacleTracker"
        root_dir : "/apollo/modules/perception/production/data/perception/camera/models/new_obstacle_tracker"
        config_file : "*.pt"
    }
    }
    ```

2. 若需要对步骤1中 `tracker_param` 的结构更新，或需要新增其他 `_param`，可在 `modules/perception/camera/app/proto/perception.proto` 文件中操作:

    ```protobuf
    message PluginParam {
    optional string name = 1;
    optional string root_dir = 2;
    optional string config_file = 3;
    }
    message TrackerParam {
    optional PluginParam plugin_param = 1;
    }
    ```

3. 若步骤1中不直接使用 `obstacle.pt` 文件，而使用其他新建的 `*.pt` 文件，则需要更改 `modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`. 其对应的 `proto` 文件为 `modules/perception/onboard/proto/fusion_camera_detection_component.proto`：

    ```protobuf
    camera_obstacle_perception_conf_dir : "/apollo/modules/perception/production/conf/perception/camera"
    camera_obstacle_perception_conf_file : "NewObstacleTracker.pt"
    ```

在完成以上步骤后，您的新camera匹配算法便可在Apollo系统中生效。
