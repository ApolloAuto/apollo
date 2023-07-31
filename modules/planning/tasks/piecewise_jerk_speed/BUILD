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
#     data_dest = "planning-task-piecewise-jerk-speed",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-piecewise-jerk-speed.BUILD",
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
#     plugin = ":libpiecewise_jerk_speed_optimizer.so",
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
#     dest = "planning-task-piecewise-jerk-speed/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-piecewise-jerk-speed/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_piecewise_jerk_speed_optimizer_description",
    description = ":plugins.xml",
    plugin = ":libpiecewise_jerk_speed_optimizer.so",
)

cc_binary(
    name = "libpiecewise_jerk_speed_optimizer.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":piecewise_jerk_speed_optimizer_lib"],
)

cc_library(
    name = "piecewise_jerk_speed_optimizer_lib",
    srcs = ["piecewise_jerk_speed_optimizer.cc"],
    hdrs = ["piecewise_jerk_speed_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_base/common:speed_profile_generator",
        "//modules/planning/planning_base/common:st_graph_data",
        "//modules/planning/planning_base/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/planning_base/task_base/common:speed_optimizer",
        "//modules/planning/tasks/piecewise_jerk_speed/proto:piecewise_jerk_speed_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
