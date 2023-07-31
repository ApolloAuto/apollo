load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]
FOPENMP_COPTS = ["-fopenmp"]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)
# install(
#     name = "install",
#     data_dest = "planning-task-open-space-trajectory-provider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-open-space-trajectory-provider.BUILD",
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
#     plugin = ":libopen_space_trajectory_provider.so",
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
#     dest = "planning-task-open-space-trajectory-provider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-open-space-trajectory-provider/include",
#     filter = "*.h",
# )

cyber_plugin_description(
    name = "plugin_open_space_trajectory_provider_description",
    description = ":plugins.xml",
    plugin = ":libopen_space_trajectory_provider.so",
)

cc_binary(
    name = "libopen_space_trajectory_provider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":open_space_trajectory_provider_lib"],
)

cc_library(
    name = "open_space_trajectory_provider_lib",
    srcs = ["open_space_trajectory_provider.cc"],
    hdrs = ["open_space_trajectory_provider.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":open_space_trajectory_optimizer",
        "//modules/common/status",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:trajectory_stitcher",
        "//modules/planning/planning_base/common/trajectory:discretized_trajectory",
        "//modules/planning/planning_base/task_base:task",
        "//modules/planning/planning_base/task_base/common:trajectory_optimizer",
        "//modules/planning/tasks/open_space_trajectory_provider/proto:open_space_trajectory_provider_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "open_space_trajectory_optimizer",
    srcs = ["open_space_trajectory_optimizer.cc"],
    hdrs = ["open_space_trajectory_optimizer.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/open_space/coarse_trajectory_generator:hybrid_a_star",
        "//modules/planning/planning_base/open_space/trajectory_smoother:distance_approach_problem",
        "//modules/planning/planning_base/open_space/trajectory_smoother:dual_variable_warm_start_problem",
        "//modules/planning/planning_base/open_space/trajectory_smoother:iterative_anchoring_smoother",
        "//modules/planning/tasks/open_space_trajectory_provider/proto:open_space_trajectory_provider_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_package()
cpplint()
