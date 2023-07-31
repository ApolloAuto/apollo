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
#     data_dest = "planning-task-lane-borrow-path",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-lane-borrow-path.BUILD",
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
#     plugin = ":liblane_borrow_path.so",
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
#     dest = "planning-task-lane-borrow-path/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-lane-borrow-path/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_lane_borrow_path_description",
    description = ":plugins.xml",
    plugin = ":liblane_borrow_path.so",
)

cc_binary(
    name = "liblane_borrow_path.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":lane_borrow_path_lib"],
)

cc_library(
    name = "lane_borrow_path_lib",
    srcs = ["lane_borrow_path.cc"],
    hdrs = ["lane_borrow_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/map/pnc_map",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:obstacle_blocking_analyzer",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/task_base/common:path_generation",
        "//modules/planning/tasks/lane_borrow_path/proto:lane_borrow_path_cc_proto",
        "//modules/planning/planning_base/task_base/common/path_util:path_util"
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
