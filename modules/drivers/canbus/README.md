# drivers-camera

## Introduction
The driver-canbus module mainly provides an interface for obtaining CAN bus data from the chassis and provides native information to the canbus module.

## Directory Structure

```shell
modules/drivers/canbus/
├── BUILD
├── can_client                      // can client implementation
│   ├── can_client_factory.cc
│   ├── can_client_factory.h
│   ├── can_client_factory_test.cc
│   ├── can_client.h
│   ├── can_client_tool.cc
│   ├── esd
│   ├── fake
│   ├── hermes_can
│   └── socket
├── can_comm                        // can receiver implementation
│   ├── can_receiver.h
│   ├── can_receiver_test.cc
│   ├── can_sender.h
│   ├── can_sender_test.cc
│   ├── message_manager.h
│   ├── message_manager_test.cc
│   ├── protocol_data.h
│   └── protocol_data_test.cc
├── common                          // common data structure
│   ├── byte.cc
│   ├── byte.h
│   ├── byte_test.cc
│   └── canbus_consts.h
├── cyberfile.xml
├── proto
├── README.md
├── sensor_canbus.h
├── sensor_gflags.cc
└── sensor_gflags.h
```

