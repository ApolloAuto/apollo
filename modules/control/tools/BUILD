load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "pad_terminal",
    srcs = ["terminal.cc"],
    deps = [
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/control/common",
        "//modules/control/proto:control_proto",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "control_tester",
    srcs = ["control_tester.cc"],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/control/common:control_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
        "@ros//:ros_common",
    ],
)

cpplint()
