load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "guardian_lib",
    srcs = ["guardian.cc"],
    hdrs = ["guardian.h"],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/guardian/common:guardian_gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/control/proto:control_proto",
        "//modules/monitor/proto:system_status_proto",
        "//modules/guardian/proto:guardian_proto",
    ],
)

cc_binary(
    name = "guardian",
    srcs = ["main.cc"],
    deps = [
        ":guardian_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cpplint()
