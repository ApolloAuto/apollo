load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_lib",
    srcs = ["canbus.cc"],
    hdrs = ["canbus.h"],
    deps = [
        "//modules/common/canbus/can_client:can_client_factory",
        "//modules/common/canbus/can_comm:can_receiver",
        "//modules/common/canbus/can_comm:can_sender",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor",
        "//modules/canbus/common:canbus_common",
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
        "//modules/common/canbus/can_client",
        "//modules/common/canbus/common:canbus_common",
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "//modules/common/monitor",
        "@ros//:ros_common",
    ],
)

filegroup(
    name = "canbus_testdata",
    srcs = glob(["testdata/**"]),
)

cpplint()
