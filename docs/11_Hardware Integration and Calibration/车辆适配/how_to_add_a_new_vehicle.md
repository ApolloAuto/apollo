# How to Add a New Vehicle to Apollo

## Introduction
The instructions below demonstrate how to add a new vehicle to Apollo.
```
Note:  The Apollo control algorithm is configured for the default vehicle, which is a Lincoln MKZ.
```

When adding a new vehicle, if your vehicle requires different attributes from those offered by the Apollo control algorithm, consider:

- Using a different control algorithm that is appropriate for your vehicle.
- Modifying the existing algorithm's parameters to achieve better results.

## Adding a New Vehicle
Complete the following task sequence to add a new vehicle:

* Implement the new vehicle controller.
* Implement the new message manager.
* Implement the new vehicle factory.
* Update the configuration file.

### Implement the New Vehicle Controller
The new vehicle controller is inherited from the  `VehicleController` class. An example header file is provided below.
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
### Implement the New Message Manager

The new message manager is inherited from the `MessageManager` class. An example header file is provided below.
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

### Implement the New Vehicle Factory Class.
The new vehicle factory class is inherited from the `AbstractVehicleFactory` class.  An example header file is provided below.
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
An example .cc file is provided below.
```cpp
std::unique_ptr<VehicleController>
NewVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController>(new lincoln::LincolnController());
}

std::unique_ptr<MessageManager> NewVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager>(new lincoln::LincolnMessageManager());
}
```

Apollo provides the base class `ProtocolData` that can be used to implement the protocols of the new vehicle.

### Register the New Vehicle

Register the new vehicle in `modules/canbus/vehicle/vehicle_factory.cc`. An example header file is provided below.
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
### Update the config File
Update the config file `modules/canbus/conf/canbus_conf.pb.txt` to activate the new vehicle in the Apollo system.
```config
vehicle_parameter {
  brand: NEW_VEHICLE_BRAND
  // put other parameters below
  ...
}
```
