load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files")
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
#     data_dest = "planning-task-rule-based-stop-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-rule-based-stop-decider.BUILD",
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
#     plugin = ":librule_based_stop_decider.so",
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
#     dest = "planning-task-rule-based-stop-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-rule-based-stop-decider/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_rule_based_stop_decider_description",
    description = ":plugins.xml",
    plugin = ":librule_based_stop_decider.so",
)

cc_binary(
    name = "librule_based_stop_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":rule_based_stop_decider_lib"],
)

cc_library(
    name = "rule_based_stop_decider_lib",
    srcs = ["rule_based_stop_decider.cc"],
    hdrs = ["rule_based_stop_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/planning/planning_base/task_base:task",
        "//modules/planning/planning_base/task_base/common/lane_change_util:lane_change_util",
        "//modules/planning/planning_base/task_base/common:decider_base",
        "//modules/planning/tasks/rule_based_stop_decider/proto:rule_based_stop_decider_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
