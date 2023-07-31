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
#     data_dest = "planning-task-path-reference-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-path-reference-decider.BUILD",
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
#     plugin = ":libpath_reference_decider.so",
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
#     dest = "planning-task-path-reference-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-path-reference-decider/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_path_reference_decider_description",
    description = ":plugins.xml",
    plugin = ":libpath_reference_decider.so",
)

cc_binary(
    name = "libpath_reference_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":path_reference_decider_lib"],
)

cc_library(
    name = "path_reference_decider_lib",
    srcs = ["path_reference_decider.cc"],
    hdrs = ["path_reference_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:path_decision",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/task_base:task",
        "//modules/planning/tasks/path_reference_decider/proto:path_reference_decider_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "path_reference_decider_test",
    size = "small",
    srcs = ["path_reference_decider_test.cc"],
    deps = [
        "path_reference_decider_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
