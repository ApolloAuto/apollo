load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

TASK_MANAGER_COPTS = ['-DMODULE_NAME=\\"task_manager\\"']

apollo_cc_library(
    name = "apollo_task_manager",
    srcs = [
        "common/task_manager_gflags.cc",
        "cycle_routing_manager.cc",
        "parking_routing_manager.cc",
        "task_manager_component.cc",
    ],
    hdrs = [
        "common/task_manager_gflags.h",
        "cycle_routing_manager.h",
        "parking_routing_manager.h",
        "task_manager_component.h",
    ],
    copts = TASK_MANAGER_COPTS,
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/task_manager_msgs:task_manager_cc_proto",
        "//modules/dreamview/backend:apollo_dreamview_backend",
        "//modules/task_manager/proto:task_manager_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_component(
    name = "libtask_manager_component.so",
    srcs = ["task_manager_component.cc"],
    hdrs = ["task_manager_component.h"],
    copts = TASK_MANAGER_COPTS,
    deps = [
        ":apollo_task_manager",
        "//cyber",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/task_manager/proto:task_manager_config_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.txt",
        "dag/*.dag",
    ]),
)

apollo_package()

cpplint()
