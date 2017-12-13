load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_checker_factory",
    srcs = ["can_checker_factory.cc"],
    hdrs = ["can_checker_factory.h"],
    deps = [
        "//modules/common:macro",
        "//modules/common/util:factory",
        "//modules/drivers/canbus/proto:canbus_proto",
        "//modules/monitor/hardware:hardware_checker",
        "//modules/monitor/hardware/can/socketcan:socketcan_checker_lib",
    ] + select({
        "//tools/platforms:use_esd_can": [
            "//modules/monitor/hardware/can/esdcan:esdcan_checker_lib",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "can_monitor",
    srcs = ["can_monitor.cc"],
    hdrs = ["can_monitor.h"],
    deps = [
        ":can_checker_factory",
        "//external:gflags",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cpplint()
