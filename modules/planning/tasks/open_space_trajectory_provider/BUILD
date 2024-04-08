load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

FOPENMP_COPTS = ["-fopenmp"]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
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

apollo_plugin(
    name = "libopen_space_trajectory_provider.so",
    srcs = [
        "open_space_trajectory_optimizer.cc",
        "open_space_trajectory_provider.cc",
    ],
    hdrs = [
        "open_space_trajectory_optimizer.h",
        "open_space_trajectory_provider.h",
    ],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/open_space_trajectory_provider/proto:open_space_trajectory_provider_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_package()

cpplint()
