load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

GUARDIAN_COPTS = ['-DMODULE_NAME=\\"guardian\\"']

apollo_component(
    name = "libguardian_component.so",
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

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()
cpplint()
