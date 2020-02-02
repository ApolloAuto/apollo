load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "control_tester",
    srcs = ["control_tester.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
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
    srcs = ["pad_terminal.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/control/common",
        "//modules/control/proto:control_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
