load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_inc_library(
    name = "hw_checker",
    hdrs = [
        "hw_checker.h",
        "annotations.h",
    ],
    deps = [
        "//modules/monitor/proto:system_status_proto",
    ]
)

cc_library(
    name = "can_checker_factory",
    srcs = ["can_checker_factory.cc"],
    hdrs = ["can_checker_factory.h"],
    deps = [
        "//modules/common:macro",
        "//modules/common/util:factory",
        "//modules/drivers/canbus/proto:canbus_proto",
        "//modules/monitor/common:hw_checker",
        "//modules/monitor/hwmonitor/hw/socketcan:socketcan_checker_lib",
      ] + select({
            "//tools/platforms:use_esd_can": [
                "//modules/monitor/hwmonitor/hw/esdcan:esdcan_checker_lib",
            ],
            "//conditions:default": [],
        }),
)

cpplint()
