load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "control_tester",
    srcs = ["control_tester.cc"],
    deps = [
        "//framework:cybertron",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
    ],
)


cc_binary(
    name = "pad_terminal",
    srcs = ["terminal.cc"],
    deps = [
        "//framework:cybertron",
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/util",
        "//modules/control/common",
        "//modules/control/proto:control_proto",
        "//modules/common/adapters:adapter_gflags",
    ],
)

cpplint()
