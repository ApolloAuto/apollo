load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "libdebug_info_control_task.so",
    description = ":plugins.xml",
    srcs = ["debug_info_control_task.cc"],
    hdrs = ["debug_info_control_task.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/filters",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:common_util",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_task_base_extend:control_task_extend",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
    ],
)

filegroup(
    name = "runtime",
    srcs = glob(["conf/*"])
)

apollo_package(enable_source=False)
cpplint()
