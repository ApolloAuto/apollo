load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_test", "apollo_cc_library", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_cc_library(
    name = "control_task_extend",
    srcs = ["control_task_extend.cc"],
    hdrs = ["control_task_extend.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_task_base_extend/common:controllers_control_task_base_extend_common",
        "//cyber",
        "//modules/common/status",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/control/control_component/proto:control_debug_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "@com_google_absl//:absl",
    ],
)

apollo_package(enable_source=False)
cpplint()