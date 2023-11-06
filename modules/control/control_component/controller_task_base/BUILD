load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_cc_library(
    name = "control_task_agent",
    srcs = ["control_task_agent.cc"],
    hdrs = ["control_task_agent.h"],
    copts = CONTROL_COPTS,
    deps = [
        "control_task",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/proto:pipeline_cc_proto",
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "@osqp",
    ],
)

apollo_cc_library(
    name = "control_task",
    srcs = [],
    hdrs = ["control_task.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
    ]),
)

apollo_package()

cpplint()
