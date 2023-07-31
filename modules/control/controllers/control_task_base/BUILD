load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_binary(
    name = "libcontrol_task_agent.so",
    srcs = ["control_task_agent.cc", "control_task_agent.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "control_task",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/control/controllers/control_task_base/common",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/control/control_component/proto:pipeline_cc_proto",
        "//modules/control/control_component/common:control_gflags",
        "@osqp",
    ],
)

cc_library(
    name = "control_task_agent",
    srcs = ["libcontrol_task_agent.so"],
    hdrs = ["control_task_agent.h"],
    copts = CONTROL_COPTS,
    deps = [
        "control_task",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/control/controllers/control_task_base/common",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/control/control_component/proto:pipeline_cc_proto",
        "//modules/control/control_component/common:control_gflags",
        "@osqp",
    ],
)

cc_binary(
    name = "libcontrol_task.so",
    srcs = ["control_task.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//modules/control/controllers/control_task_base/common",
        "//modules/control/controllers/control_task_base/proto:calibration_table_cc_proto",
        "//cyber/plugin_manager:plugin_manager",
    ],
)

cc_library(
    name = "control_task",
    srcs = ["libcontrol_task.so"],
    hdrs = ["control_task.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/controllers/control_task_base/common",
        "//modules/control/controllers/control_task_base/proto:calibration_table_cc_proto",
        "//cyber/plugin_manager:plugin_manager",
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
