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

* Register the New Vehicle brand.
* Generate the code for the New Vehicle.
* Update proto and BUILD files.
* Register the New Vehicle.
* Update the configuration file.

### Register the New Vehicle brand
The new vehicle brand must be located in `modules/common/configs/proto/vehicle_config.proto`.
```proto
...
enum VehicleBrand {
  LINCOLN_MKZ = 0;
  // New vehicle brand below
  NEW_VEHICLE_BRAND = 1;
}
...
```

### Generate the code for the New Vehicle
There is a handy tool [gen_vehicle_protocol]((https://github.com/ApolloAuto/apollo/tree/master/modules/tools/gen_vehicle_protocol)) that generates almost complete code for a new vehicle. Move the generated code for the new vehicle to the canbus module.

### Update proto and BUILD files
Include the new vehicle proto file to `chassis_detail.proto`.
```proto
syntax = "proto2";

package apollo.canbus;

import "modules/common/configs/proto/vehicle_config.proto";
import "modules/canbus/proto/chassis.proto";
import "modules/canbus/proto/new_vehicle.proto";

message ChassisDetail {
  ...
  // Add new vehicle
  optional New_vehicle new_vehicle = 18;
}
...
```

Generate proto BUILD file.
```sh
$ cd /apollo
$ python3 ./scripts/proto_build_generator.py modules/canbus/proto/BUILD
```

### Register the New Vehicle

Register the new vehicle in `modules/canbus/vehicle/vehicle_factory.cc` and update `modules/canbus/vehicle/BUILD`. Examples are provided below.

```cpp
...
#include "modules/canbus/vehicle/new_vehicle/new_vehicle_vehicle_factory.h"
...
void VehicleFactory::RegisterVehicleFactory() {
  Register(VehicleParameter::LINCOLN_MKZ, []() -> AbstractVehicleFactory* {
    return new LincolnVehicleFactory();
  });

  // register the new vehicle here.
  Register(VehicleParameter::NEW_VEHICLE_BRAND, []() -> AbstractVehicleFactory* {
    return new New_vehicleVehicleFactory();
  });
}
...
```

```py
...
cc_library(
    name = "vehicle_factory",
    srcs = ["vehicle_factory.cc"],
    hdrs = ["vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":abstract_vehicle_factory",
        "//modules/common/util:factory",
        "//modules/canbus/vehicle/lincoln:lincoln_vehicle_factory",
        # New vehicle here.
        "//modules/canbus/vehicle/new_vehicle:new_vehicle_vehicle_factory",
    ],
)
...
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
