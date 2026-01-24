load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "libanti_slip_control_task.so",
    description = ":plugins.xml",
    srcs = ["anti_slip_control_task.cc"],
    hdrs = ["anti_slip_control_task.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/filters",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/control/control_component/common:control_gflags",
	    "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_component/controller_task_base/common:interpolation_1d",
        "//modules/control/control_component/controller_task_base/common:interpolation_2d",
        "//modules/control/control_task_base_extend:control_task_extend",
        "//modules/control/control_task_base_extend/common:controllers_control_task_base_extend_common",
        "//modules/control/controllers/slope_anti_slip_control_task/proto:antislip_control_task_conf_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/control/control_task_base_extend/proto:params_pipeline_proto",
    ],
)

filegroup(
    name = "runtime",
    srcs = glob(["conf/*"])
)

apollo_package(enable_source=False)
cpplint()
