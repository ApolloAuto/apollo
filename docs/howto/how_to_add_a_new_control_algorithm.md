# How to Add a New Control Algorithm

The control algorithm in Apollo consists of one or more controllers that can be easily changed or replaced with different algorithms. Each controller outputs one or more control commands to `CANbus`. The default control algorithm in Apollo contains a lateral controller (LatController) and a longitudinal controller (LonController). They are responsible for the vehicle control in the lateral and longitudinal directions respectively.

The new control algorithm does not have to follow the default pattern, e.g., one lateral controller + one longitudinal controller. It could be a single controller or a combination of any number of controllers.

Complete the following task sequence to add a new control algorithm:

1. Create a controller
2. Add the new controller configuration into the `control_config` file
3. Register the new controller

The steps are elaborated below for better understanding:

## Create a Controller

All controllers must inherit the base class `Controller`, which defines a set of interfaces. Here is an example of a controller implementation:

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



## Add a New Controller Configuration to the control_config File

To add the new controller configuration complete the following steps:

1. Define a `proto` for the new controller configurations and parameters based on the algorithm requirements. An example `proto` definition of `LatController` can be found at:  `modules/control/proto/lat_controller_conf.proto`
2. After defining the new controller `proto`, e.g., `new_controller_conf.proto`, type the following:

    ```protobuf
    syntax = "proto2";

    package apollo.control;

    message NewControllerConf {
        double parameter1 = 1;
        int32 parameter2 = 2;
    }
    ```

3. Update `control_conf.proto` at  `modules/control/proto/control_conf.proto`:

    ```protobuf
    optional apollo.control.NewControllerConf new_controller_conf = 15;
    ```

4. Update `ControllerType` in this file:

    ```protobuf
    enum ControllerType {
        LAT_CONTROLLER = 0;
        LON_CONTROLLER = 1;
        NEW_CONTROLLER = 2;
      };
    ```

5. When the `protobuf` definition is complete, update the control configuration file accordingly at `modules/control/conf/control_conf.pb.txt`

```
Note: The above `control/conf` file is the default for Apollo.  Your project may use a different control configuration file.
```

## Register the New Controller

To activate a new controller in the Apollo system, register the new controller in `ControllerAgent`.  Go to:

> modules/control/controller/controller_agent.cc

Type your registration information in the shell. For example:

```c++
void ControllerAgent::RegisterControllers() {
  controller_factory_.Register(
      ControlConf::NEW_CONTROLLER,
      []() -> Controller * { return new NewController(); });
}
```

After this code update sequence is complete, you new controller should take effect in the Apollo system.
