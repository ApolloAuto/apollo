# Canbus

## Introduction
Canbus accepts and executes control module commands and collects the car's chassis status as feedback to control.

## Input
  * Control commands

## Output
  * Chassis status
  * Chassis detailed status

## Implementation

The major components in canbus module are:
  * **Vehicle**: the vehicle itself, including its controller and message manager

  * **CAN Client** - CAN client has been moved to `/modules/drivers/canbus` since it is shared by different sensors utilizing the canbus protocol

You can implement your own CAN client in the folder `can_client` by inheriting from the `CanClient` class.

```
Note:
Do not forget to register your CAN client in `CanClientFactory`.
```

You can also implement your own vehicle controller and message manager in the folder `vehicle` by inheriting from `VehicleController` and `MessageManager`.

```
Note:
Do not forget to register your vehicle in `VehicleFactory`.
```