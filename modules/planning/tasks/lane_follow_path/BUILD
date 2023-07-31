load("@rules_cc//cc:defs.bzl", "cc_library")
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
#     data_dest = "planning-task-lane-follow-path",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-lane-follow-path.BUILD",
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
#     plugin = ":liblane_follow_path.so",
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
#     dest = "planning-task-lane-follow-path/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-lane-follow-path/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_lane_follow_path_description",
    description = ":plugins.xml",
    plugin = ":liblane_follow_path.so",
)

cc_binary(
    name = "liblane_follow_path.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":lane_follow_path_lib"],
)

cc_library(
    name = "lane_follow_path_lib",
    srcs = ["lane_follow_path.cc"],
    hdrs = ["lane_follow_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/map/pnc_map",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/task_base/common:path_generation",
        "//modules/planning/tasks/lane_follow_path/proto:lane_follow_path_cc_proto",
        "//modules/planning/planning_base/task_base/common/path_util:path_util"
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
