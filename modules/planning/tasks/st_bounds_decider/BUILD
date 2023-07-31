load("@rules_cc//cc:defs.bzl", "cc_library")
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
#     data_dest = "planning-task-st-bounds-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-st-bounds-decider.BUILD",
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
#     plugin = ":libst_bounds_decider.so",
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
#     dest = "planning-task-st-bounds-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-st-bounds-decider/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_st_bounds_decider_description",
    description = ":plugins.xml",
    plugin = ":libst_bounds_decider.so",
)

cc_binary(
    name = "libst_bounds_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":st_bounds_decider_lib"],
)

cc_library(
    name = "st_bounds_decider_lib",
    srcs = ["st_bounds_decider.cc"],
    hdrs = ["st_bounds_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_driving_limits",
        ":st_guide_line",
        ":st_obstacles_processor",
        "//modules/common/status",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:st_graph_data",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/planning/planning_base/task_base:task",
        "//modules/planning/planning_base/task_base/common:decider_base",
        "//modules/planning/tasks/st_bounds_decider/proto:st_bounds_decider_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "st_guide_line",
    srcs = ["st_guide_line.cc"],
    hdrs = ["st_guide_line.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:obstacle",
        "//modules/planning/planning_base/common:path_decision",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:speed_limit",
        "//modules/planning/planning_base/common/path:discretized_path",
        "//modules/planning/planning_base/common/path:frenet_frame_path",
        "//modules/planning/planning_base/common/path:path_data",
        "//modules/planning/planning_base/common/speed:speed_data",
        "//modules/planning/planning_base/common/speed:st_boundary",
        "//modules/planning/planning_base/common/trajectory:discretized_trajectory",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/reference_line",
    ],
)

cc_library(
    name = "st_driving_limits",
    srcs = ["st_driving_limits.cc"],
    hdrs = ["st_driving_limits.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:obstacle",
        "//modules/planning/planning_base/common:path_decision",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:speed_limit",
        "//modules/planning/planning_base/common/path:discretized_path",
        "//modules/planning/planning_base/common/path:frenet_frame_path",
        "//modules/planning/planning_base/common/path:path_data",
        "//modules/planning/planning_base/common/speed:st_boundary",
        "//modules/planning/planning_base/common/trajectory:discretized_trajectory",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/reference_line",
    ],
)

cc_library(
    name = "st_obstacles_processor",
    srcs = ["st_obstacles_processor.cc"],
    hdrs = ["st_obstacles_processor.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:history",
        "//modules/planning/planning_base/common:obstacle",
        "//modules/planning/planning_base/common:path_decision",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:speed_limit",
        "//modules/planning/planning_base/common/path:discretized_path",
        "//modules/planning/planning_base/common/path:frenet_frame_path",
        "//modules/planning/planning_base/common/path:path_data",
        "//modules/planning/planning_base/common/speed:st_boundary",
        "//modules/planning/planning_base/common/trajectory:discretized_trajectory",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/reference_line",
    ],
)

apollo_package()
cpplint()
