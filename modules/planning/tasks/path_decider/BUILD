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
#     data_dest = "planning-task-path-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-path-decider.BUILD",
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
#     plugin = ":libpath_decider.so",
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
#     dest = "planning-task-path-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-path-decider/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_path_decider_description",
    description = ":plugins.xml",
    plugin = ":libpath_decider.so",
)

cc_binary(
    name = "libpath_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":path_decider_lib"],
)

cc_library(
    name = "path_decider_lib",
    srcs = ["path_decider.cc"],
    hdrs = ["path_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common_msgs/planning_msgs:decision_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/tasks/path_decider/proto:path_decider_cc_proto",
        "//modules/planning/planning_base/task_base:task",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
