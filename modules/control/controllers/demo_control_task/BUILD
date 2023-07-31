load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

filegroup(
    name = "runtime",
    srcs = glob(["conf/*"])
)

cyber_plugin_description(
    name = "plugin_demo_control_task_description",
    description = ":plugins.xml",
    plugin = ":libdemo_control_task.so",
)

cc_binary(
    name = "libdemo_control_task.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"control\\\""],
    deps = [":demo_control_task_lib"],
)

cc_library(
    name = "demo_control_task_lib",
    srcs = ["demo_control_task.cc"],
    hdrs = ["demo_control_task.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/controllers/control_task_base:control_task",
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
    alwayslink = True,
)

apollo_package()
cpplint()