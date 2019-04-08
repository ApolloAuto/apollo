# 如何在Apollo中添加新的车辆

## 简介
本文描述了如何向Apollo中添加新的车辆。

```
注意: Apollo控制算法将林肯MKZ配置为默认车辆
```

添加新的车辆时，如果您的车辆需要不同于Apollo控制算法提供的属性，请参考：

- 使用适合您的车辆的其它控制算法。
- 修改现有算法的参数以获得更好的结果。

## 增加新车辆 

添加新车辆需要完成以下任务： 

* 实现新的车辆控制器

* 实现新的消息管理器

* 实现新的车辆工厂类

* 更新配置文件

### 实现新的车辆控制器
新的车辆控制器是从 `VehicleController`类继承的。 下面提供了一个头文件示例。
```cpp
/**
 * @class NewVehicleController
 *
 * @brief this class implements the vehicle controller for a new vehicle.
 */
class NewVehicleController final : public VehicleController {
 public:
  /**
   * @brief initialize the new vehicle controller.
   * @return init error_code
   */
  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params, CanSender* const can_sender,
      MessageManager* const message_manager) override;

  /**
   * @brief start the new vehicle controller.
   * @return true if successfully started.
   */
  bool Start() override;

  /**
   * @brief stop the new vehicle controller.
   */
  void Stop() override;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  Chassis chassis() override;

  // more functions implemented here
  ...

};
```
### 实现新的消息管理器
新的消息管理器是从 `MessageManager` 类继承的。 下面提供了一个头文件示例。
```cpp
/**
 * @class NewVehicleMessageManager
 *
 * @brief implementation of MessageManager for the new vehicle
 */
class NewVehicleMessageManager : public MessageManager {
 public:
  /**
   * @brief construct a lincoln message manager. protocol data for send and
   * receive are added in the construction.
   */
  NewVehicleMessageManager();
  virtual ~NewVehicleMessageManager();

  // define more functions here.
  ...
};
```

### 实现新的车辆工厂类
新的车辆工厂是从 `AbstractVehicleFactory` 类继承的。下面提供了一个头文件示例。
```cpp
/**
 * @class NewVehicleFactory
 *
 * @brief this class is inherited from AbstractVehicleFactory. It can be used to
 * create controller and message manager for lincoln vehicle.
 */
class NewVehicleFactory : public AbstractVehicleFactory {
 public:
  /**
  * @brief destructor
  */
  virtual ~NewVehicleFactory() = default;

  /**
   * @brief create lincoln vehicle controller
   * @returns a unique_ptr that points to the created controller
   */
  std::unique_ptr<VehicleController> CreateVehicleController() override;

  /**
   * @brief create lincoln message manager
   * @returns a unique_ptr that points to the created message manager
   */
  std::unique_ptr<MessageManager> CreateMessageManager() override;
};
```
一个.cc示例文件如下：
```cpp
std::unique_ptr<VehicleController>
NewVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController>(new lincoln::LincolnController());
}

std::unique_ptr<MessageManager> NewVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager>(new lincoln::LincolnMessageManager());
}
```

Apollo提供可以用于实现新的车辆协议的基类 `ProtocolData`。

### 更新配置文件

在`modules/canbus/vehicle/vehicle_factory.cc`里注册新的车辆。 下面提供了一个头文件示例。
```cpp
void VehicleFactory::RegisterVehicleFactory() {
  Register(VehicleParameter::LINCOLN_MKZ, []() -> AbstractVehicleFactory* {
    return new LincolnVehicleFactory();
  });

  // register the new vehicle here.
  Register(VehicleParameter::NEW_VEHICLE_BRAND, []() -> AbstractVehicleFactory* {
    return new NewVehicleFactory();
  });
}
```
### 更新配置文件
在 `modules/canbus/conf/canbus_conf.pb.txt` 中更新配置，在Apollo系统中激活车辆。
```config
vehicle_parameter {
  brand: NEW_VEHICLE_BRAND
  // put other parameters below
  ...
}
```