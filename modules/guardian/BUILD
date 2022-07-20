load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

GUARDIAN_COPTS = ['-DMODULE_NAME=\\"guardian\\"']

cc_library(
    name = "guardian_component_lib",
    srcs = ["guardian_component.cc"],
    hdrs = ["guardian_component.h"],
    copts = GUARDIAN_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/guardian_msgs:guardian_cc_proto",
        "//modules/guardian/proto:guardian_conf_cc_proto",
        "//modules/common_msgs/monitor_msgs:system_status_cc_proto",
    ],
)

cc_binary(
    name = "libguardian_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":guardian_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libguardian_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
