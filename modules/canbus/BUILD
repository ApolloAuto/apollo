load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_lib",
    srcs = ["canbus.cc"],
    hdrs = ["canbus.h"],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
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
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "//modules/common/monitor",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/common:canbus_common",
        "@ros//:ros_common",
    ],
)

filegroup(
    name = "canbus_testdata",
    srcs = glob(["testdata/**"]),
)

cpplint()
