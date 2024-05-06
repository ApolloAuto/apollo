load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
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

apollo_plugin(
    name = "libopen_space_pre_stop_decider.so",
    srcs = ["open_space_pre_stop_decider.cc"],
    hdrs = ["open_space_pre_stop_decider.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/tasks/open_space_pre_stop_decider/proto:open_space_pre_stop_decider_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
    ],
)

apollo_package()

cpplint()
