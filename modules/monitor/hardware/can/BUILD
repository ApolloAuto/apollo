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
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/common:canbus_common",
        "//modules/common/util:util",
        "//modules/monitor/common:monitor",
    ],
)

cc_binary(
    name = "can_check",
    srcs = [
        "can_check.cc",
    ],
    deps = [
        ":can_checker_factory",
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/common:canbus_common",
        "//modules/common/util:util",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hardware:hardware_check_utils",
    ],
)

cc_binary(
    name = "mock_can_check",
    srcs = [
        "can_check.cc",
    ],
    defines = ["DEBUG"],
    deps = [
        ":can_checker_factory",
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/common:canbus_common",
        "//modules/common/util:util",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hardware:hardware_check_utils",
    ],
)

cpplint()
