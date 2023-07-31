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
#     data_dest = "planning-task-open-space-pre-stop-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-open-space-pre-stop-decider.BUILD",
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
#     plugin = ":libopen_space_pre_stop_decider.so",
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
#     dest = "planning-task-open-space-pre-stop-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-open-space-pre-stop-decider/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_open_space_pre_stop_decider_description",
    description = ":plugins.xml",
    plugin = ":libopen_space_pre_stop_decider.so",
)

cc_binary(
    name = "libopen_space_pre_stop_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":open_space_pre_stop_decider_lib"],
)

cc_library(
    name = "open_space_pre_stop_decider_lib",
    srcs = ["open_space_pre_stop_decider.cc"],
    hdrs = ["open_space_pre_stop_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/planning/planning_base/task_base/common:decider_base",
        "//modules/planning/tasks/open_space_pre_stop_decider/proto:open_space_pre_stop_decider_cc_proto"
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
