# Customize Your Own HMI

HMIWorker is a standalone singleton which processes HMI actions. If you want to
have a customized HMI instead the one integrated with Apollo Dreamview, just
develope a frontend and send operations to the backend which delegates to
HMIWorker.

## HMI Config

See modules/dreamview/proto/hmi_config.proto. It defines all supported modes,
modules, hardware and tools.

## HMI Worker

According to the HMIConfig, HMI Worker could trigger actions like:
- Change mode, map, vehicle and driving mode.
- Register event handler for changing mode, map and vehicle.
- Start, stop or execute other registered commands for modules.
- Execute registered tools.
- Submit DriveEvent which will be recorded as a ROS message.
- Get current HMIConfig and HMIStatus, which could be used for UI update.

## Vehicle Manager

VehicleManager is the one to actually config everything when changing vehicles.
The major thing is to copy all vehicle-wised parameter files to right place. The
source and destination mappings are defined by
modules/dreamview/conf/vehicle_data.pb.txt.
