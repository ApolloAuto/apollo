load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "canbus/bin",
    targets = [
        ":canbus_tester",
        ":teleop",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "canbus_tester",
    srcs = ["canbus_tester.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:util_tool",
    ],
)

cpplint()
