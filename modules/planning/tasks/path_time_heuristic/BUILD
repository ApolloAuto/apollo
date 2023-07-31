load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]
filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)
# install(
#     name = "install",
#     data_dest = "planning-task-path-time-heuristic",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-path-time-heuristic.BUILD",
#     ],
#     deps = [
#         "install_plugin",
#     ],
# )

# install_plugin(
#     name = "install_plugin",
#     data = [
#     ] + glob(["conf/**"]),
#     description = ":plugins.xml",
#     plugin = ":libpath_time_heuristic_optimizer.so",
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_all_src",
#         ":install_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_all_src",
#     src_dir = ["."],
#     dest = "planning-task-path-time-heuristic/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-path-time-heuristic/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_path_time_heuristic_optimizer_description",
    description = ":plugins.xml",
    plugin = ":libpath_time_heuristic_optimizer.so",
)

cc_library(
    name = "st_graph_point",
    srcs = ["st_graph_point.cc"],
    hdrs = ["st_graph_point.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/speed:st_point",
    ],
)

cc_library(
    name = "dp_st_cost",
    srcs = ["dp_st_cost.cc"],
    hdrs = ["dp_st_cost.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_graph_point",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:obstacle",
        "//modules/planning/planning_base/common/speed:st_boundary",
        "//modules/planning/planning_base/task_base/utils:st_gap_estimator",
        "//modules/planning/tasks/path_time_heuristic/proto:path_time_heuristic_cc_proto",
    ],
)

cc_library(
    name = "gridded_path_time_graph",
    srcs = ["gridded_path_time_graph.cc"],
    hdrs = ["gridded_path_time_graph.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":dp_st_cost",
        ":st_graph_point",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/planning/planning_base/common:obstacle",
        "//modules/planning/planning_base/common:path_decision",
        "//modules/planning/planning_base/common:st_graph_data",
        "//modules/planning/planning_base/common/speed:speed_data",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
    ],
)

cc_binary(
    name = "libpath_time_heuristic_optimizer.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":path_time_heuristic_optimizer_lib"],
)

cc_library(
    name = "path_time_heuristic_optimizer_lib",
    srcs = ["path_time_heuristic_optimizer.cc"],
    hdrs = ["path_time_heuristic_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":dp_st_cost",
        ":gridded_path_time_graph",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/planning_base/common:st_graph_data",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/planning/tasks/speed_bounds_decider:st_boundary_mapper",
        "//modules/planning/planning_base/task_base/common:speed_optimizer",
        "//modules/planning/tasks/path_time_heuristic/proto:path_time_heuristic_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "gridded_path_time_graph_test",
    size = "small",
    srcs = ["gridded_path_time_graph_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    deps = [
        ":gridded_path_time_graph",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
