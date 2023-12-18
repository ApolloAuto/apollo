load("//tools:cpplint.bzl", "cpplint")
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

apollo_cc_test(
    name = "path_reference_decider_test",
    size = "small",
    srcs = ["path_reference_decider_test.cc"],
    deps = [
        "path_reference_decider_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_plugin(
    name = "libpath_reference_decider.so",
    srcs = ["path_reference_decider.cc"],
    hdrs = ["path_reference_decider.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/tasks/path_reference_decider/proto:path_reference_decider_cc_proto",
    ],
)

apollo_package()

cpplint()