load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

filegroup(
    name = "runtime",
    srcs = glob(["conf/*"])
)

apollo_plugin(
    name = "libdemo_control_task.so",
    srcs = ["demo_control_task.cc"],
    hdrs = ["demo_control_task.h"],
    description = ":plugins.xml",
    deps = [
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_component/common:control_gflags",
        "//cyber",
        "//modules/common/status",
        "//modules/control/controllers/demo_control_task/proto:demo_control_task_conf_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/dreamview_msgs:chart_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "@eigen",
    ],
)

apollo_package()
cpplint()