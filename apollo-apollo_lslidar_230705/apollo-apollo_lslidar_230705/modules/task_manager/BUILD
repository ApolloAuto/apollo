load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
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
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/task_manager/common:task_manager_gflags",
        "//modules/common_msgs/task_manager_msgs:task_manager_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "parking_routing_manager_lib",
    srcs = ["parking_routing_manager.cc"],
    hdrs = ["parking_routing_manager.h"],
    copts = TASK_MANAGER_COPTS,
    deps = [
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/dreamview/backend/map:map_service",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/task_manager/common:task_manager_gflags",
        "//modules/common_msgs/task_manager_msgs:task_manager_cc_proto",
        "//modules/task_manager/proto:task_manager_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "task_manager_component_lib",
    srcs = ["task_manager_component.cc"],
    hdrs = ["task_manager_component.h"],
    copts = TASK_MANAGER_COPTS,
    deps = [
        ":cycle_routing_manager_lib",
        ":parking_routing_manager_lib",
        "//cyber",
        "//modules/task_manager/proto:task_manager_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libtask_manager_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":task_manager_component_lib"],
)

install(
    name = "install",
    data_dest = "task-manager",
    library_dest = "task-manager/lib",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":task-manager.BUILD",
    ],
    targets = [
        ":libtask_manager_component.so",
    ],
    deps = [
        ":pb_hdrs",
        "//modules/task_manager/proto:py_pb_task_manager"
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "task-manager/include",
    data = [
        "//modules/task_manager/proto:task_manager_config_cc_proto",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_task_manager_src",
        ":install_task_manager_hdrs"
    ],
)

install_src_files(
    name = "install_task_manager_src",
    src_dir = ["."],
    dest = "task-manager/src",
    filter = "*",
)

install_src_files(
    name = "install_task_manager_hdrs",
    src_dir = ["."],
    dest = "task-manager/include",
    filter = "*.h",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.txt",
        "dag/*.dag",
    ]),
)

cpplint()
