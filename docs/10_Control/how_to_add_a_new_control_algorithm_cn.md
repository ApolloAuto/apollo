# 如何添加新的控制算法

Apollo中的控制算法由一个或多个控制器组成，可以轻松更改或替换为不同的算法。 每个控制器将一个或多个控制命令输出到`CANbus`。 Apollo中的默认控制算法包含横向控制器（LatController）和纵向控制器（LonController）。 它们分别负责横向和纵向的车辆控制。

新的控制算法不必遵循默认模式，例如，一个横向控制器+一个纵向控制器。 它可以是单个控制器，也可以是任意数量控制器的组合。

添加新的控制算法的步骤：

1. 创建一个控制器
2. 在文件`control_config` 中添加新控制器的配置信息
3. 注册新控制器

为了更好的理解，下面对每个步骤进行详细的阐述:

## 创建一个控制器

所有控制器都必须继承基类`Controller`，它定义了一组接口。 以下是控制器实现的示例:

```c++
namespace apollo {
namespace control {

class NewController : public Controller {
 public:
  NewController();
  virtual ~NewController();
  Status Init(const ControlConf* control_conf) override;
  Status ComputeControlCommand(
      const localization::LocalizationEstimate* localization,
      const canbus::Chassis* chassis, const planning::ADCTrajectory* trajectory,
      ControlCommand* cmd) override;
  Status Reset() override;
  void Stop() override;
  std::string Name() const override;
};
}  // namespace control
}  // namespace apollo
```



## 在文件`control_config` 中添加新控制器的配置信息

按照下面的步骤添加新控制器的配置信息:

1. 根据算法要求为新控制器配置和参数定义`proto`。作为示例，可以参考以下位置的`LatController`的`proto`定义：`modules/control/proto/ lat_controller_conf.proto`
2. 定义新的控制器`proto`之后，例如`new_controller_conf.proto`，输入以下内容:

    ```protobuf
    syntax = "proto2";

    package apollo.control;

    message NewControllerConf {
        double parameter1 = 1;
        int32 parameter2 = 2;
    }
    ```

3. 参考如下内容更新 `modules/control/proto/control_conf.proto`文件:

    ```protobuf
    optional apollo.control.NewControllerConf new_controller_conf = 15;
    ```

4. 参考以内容更新 `ControllerType`（在`modules/control/proto/control_conf.proto` 中）:

    ```protobuf
    enum ControllerType {
        LAT_CONTROLLER = 0;
        LON_CONTROLLER = 1;
        NEW_CONTROLLER = 2;
      };
    ```

5. `protobuf`定义完成后，在`modules/control/conf/control_conf.pb.txt`中相应更新控制配置文件。

```
注意：上面的"control/conf"文件是Apollo的默认文件。您的项目可能使用不同的控制配置文件.
```

## 注册新控制器

要激活Apollo系统中的新控制器，请在如下文件中的“ControllerAgent”中注册新控制器:

> modules/control/controller/controller_agent.cc

按照如下示例添加注册信息:

```c++
void ControllerAgent::RegisterControllers() {
  controller_factory_.Register(
      ControlConf::NEW_CONTROLLER,
      []() -> Controller * { return new NewController(); });
}
```

在完成以上步骤后，您的新控制器便可在Apollo系统中生效。
