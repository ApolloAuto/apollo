load("//tools:cpplint.bzl", "cpplint")

# load("//tools/install:install.bzl", "install", "install_files")

load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
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

apollo_plugin(
    name = "librule_based_stop_decider.so",
    srcs = ["rule_based_stop_decider.cc"],
    hdrs = ["rule_based_stop_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//modules/common/status",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/rule_based_stop_decider/proto:rule_based_stop_decider_cc_proto",
    ],
)

apollo_package()

cpplint()
