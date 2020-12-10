load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

TASK_MANAGER_COPTS = ['-DMODULE_NAME=\\"task_manager\\"']

cc_library(
    name = "cycle_routing_manager_lib",
    srcs = ["cycle_routing_manager.cc"],
    hdrs = ["cycle_routing_manager.h"],
    copts = TASK_MANAGER_COPTS,
    deps = [
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/dreamview/backend/map:map_service",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/task_manager/common:task_manager_gflags",
        "//modules/task_manager/proto:task_manager_cc_proto",
    ],
)

cc_library(
    name = "task_manager_component_lib",
    srcs = ["task_manager_component.cc"],
    hdrs = ["task_manager_component.h"],
    copts = TASK_MANAGER_COPTS,
    deps = [
        ":cycle_routing_manager_lib",
        "//cyber",
        "//modules/task_manager/proto:task_manager_config_cc_proto",
    ],
)

cc_binary(
    name = "libtask_manager_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":task_manager_component_lib"],
)

cpplint()
