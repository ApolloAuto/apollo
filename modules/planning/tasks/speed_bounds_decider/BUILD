load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)
# install(
#     name = "install",
#     data_dest = "planning-task-speed-bounds-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-speed-bounds-decider.BUILD",
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
#     plugin = ":libspeed_bounds_decider.so",
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
#     dest = "lanning-task-speed-bounds-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "lanning-task-speed-bounds-decider/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_speed_bounds_decider_description",
    description = ":plugins.xml",
    plugin = ":libspeed_bounds_decider.so",
)

cc_binary(
    name = "libspeed_bounds_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":speed_bounds_decider_lib"],
)

cc_library(
    name = "speed_bounds_decider_lib",
    srcs = ["speed_bounds_decider.cc"],
    hdrs = ["speed_bounds_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":st_boundary_mapper",
        "//modules/common/status",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:st_graph_data",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/planning/planning_base/task_base:task",
        "//modules/planning/planning_base/task_base/common:decider_base",
        "//modules/planning/tasks/speed_bounds_decider/proto:speed_bounds_decider_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "speed_limit_decider.cc",
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "speed_limit_decider.h",
        "st_boundary_mapper.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning/planning_base/common:dependency_injector",
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
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/reference_line",
        "//modules/planning/tasks/speed_bounds_decider/proto:speed_bounds_decider_cc_proto",
    ],
)

cc_test(
    name = "st_boundary_mapper_test",
    size = "small",
    srcs = ["st_boundary_mapper_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_testdata",
    ],
    deps = [
        ":st_boundary_mapper",
        "//cyber",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
