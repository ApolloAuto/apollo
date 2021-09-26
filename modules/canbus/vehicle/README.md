# Vehicle

## Introduction
Because the canbus commands of different types of vehicles are different, the canbus message format of different types of vehicles is adapted here.

## Vehicle Controller
All vehicles are inherited from the `VehicleController` base class, and the canbus information is sent and read through the abstraction of `VehicleController`.

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
## Message Manager
The main function of `MessageManager` is to parse and save canbus data, and the specific reception and transmission are in `CanReceiver` and `CanSender`.

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

## Vehicle Factory Class
Vehicles can be adapted to different vehicle types, and each vehicle type corresponds to a Vehicle Controller. The process of creating each vehicle controller (`VehicleController`) and message management (`MessageManager`) is as follows: The `VehicleFactory` class creates different types of `AbstractVehicleFactory` for each vehicle type.

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
  return std::unique_ptr<VehicleController>(new newvehicle::NewVehicleController());
}

std::unique_ptr<MessageManager> NewVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager>(new newvehicle::NewVehicleMessageManager());
}
```

Apollo provides the base class `ProtocolData` that can be used to implement the protocols of the new vehicle.
