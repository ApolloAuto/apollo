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
#     data_dest = "planning-task-open-space-trajectory-partition",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-open-space-trajectory-partition.BUILD",
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
#     plugin = ":libopen_space_trajectory_partition.so",
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
#     dest = "planning-task-open-space-trajectory-partition/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-open-space-trajectory-partition/include",
#     filter = "*.h",
# )

apollo_cc_test(
    name = "open_space_trajectory_partition_test",
    size = "small",
    srcs = ["open_space_trajectory_partition_test.cc"],
    deps = [
        ":open_space_trajectory_partition_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_plugin(
    name = "libopen_space_trajectory_partition.so",
    srcs = ["open_space_trajectory_partition.cc"],
    hdrs = ["open_space_trajectory_partition.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/open_space_trajectory_partition/proto:open_space_trajectory_partition_cc_proto",
    ],
)

apollo_package()

cpplint()