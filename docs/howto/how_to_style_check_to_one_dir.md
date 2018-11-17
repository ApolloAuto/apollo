## How to add C++ style check to one directory?
1. In the BUILD file, at the top, add the following line
    ```
    load("//tools:cpplint.bzl", "cpplint")
    ```
2. In the BUILD file, at the bottom, add the following line
    ```
    cpplint()
    ```
3. Run `apollo.sh check` to verify if the directory passes style check.

### Please use the [BUILD](https://github.com/ApolloAuto/apollo/blob/master/modules/canbus/BUILD) file of canbus module as an example.
```
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_lib",
    srcs = ["canbus.cc"],
    hdrs = ["canbus.h"],
    deps = [
        "//modules/canbus/can_client:can_client_factory",
        "//modules/canbus/can_comm:can_receiver",
        "//modules/canbus/can_comm:can_sender",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/hmi/utils:hmi_status_helper",
    ],
)

cc_test(
    name = "canbus_test",
    size = "small",
    srcs = ["canbus_test.cc"],
    deps = [
        "//modules/canbus:canbus_lib",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "canbus",
    srcs = ["main.cc"],
    deps = [
        ":canbus_lib",
        "//external:gflags",
        "//modules/canbus/can_client",
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//third_party/ros:ros_common",
    ],
)

filegroup(
    name = "canbus_testdata",
    srcs = glob(["testdata/**"]),
)

cpplint()
```

