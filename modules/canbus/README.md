# Canbus

## Introduction
  Canbus accepts and executes the control command, and collect chassis status as feedback to control.

## Input
  * Control command

## Output
  * Chassis status
  * Chassis detail status

## Implementation
  The major components in canbus module are:
  * Vehicle including vehicle controller and message manager

  * (CAN client has been moved to `/modules/drivers/canbus` since it is shared by different sensors utilizing canbus protocol)

  Your own CAN client can be implemented in the folder of *can_client* by inheriting from `CanClient` class. Remember to register your CAN client in `CanClientFactory`.

  Your own vehicle controller and message manager can be implemented in the folder of *vehicle* by inheriting from `VehicleController` and `MessageManager`. Remember to register your vehicle in `VehicleFactory`.
