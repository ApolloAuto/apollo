## 如何将C++样式的检查添加到一个目录？

在BUILD文件顶部，添加以下这一行：
```
load("//tools:cpplint.bzl", "cpplint")
```
在BUILD文件底部添加以下这一行：
```
cpplint()
```
运行`apollo.sh check`以验证目录是否通过样式检查。
### 请以CANBUS模块的BUILD文件为例。
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
