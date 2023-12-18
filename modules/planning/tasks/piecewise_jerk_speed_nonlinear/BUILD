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
#     data_dest = "planning-task-piecewise-jerk-speed-nonlinear",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-piecewise-jerk-speed-nonlinear.BUILD",
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
#     plugin = ":libpiecewise_jerk_speed_nonlinear_optimizer.so",
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
#     dest = "planning-task-piecewise-jerk-speed-nonlinear/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-piecewise-jerk-speed-nonlinear/include",
#     filter = "*.h",
# )

apollo_plugin(
    name = "libpiecewise_jerk_speed_nonlinear_optimizer.so",
    srcs = [
        "piecewise_jerk_speed_nonlinear_ipopt_interface.cc",
        "piecewise_jerk_speed_nonlinear_optimizer.cc",
    ],
    hdrs = [
        "piecewise_jerk_speed_nonlinear_ipopt_interface.h",
        "piecewise_jerk_speed_nonlinear_optimizer.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//modules/common/util:common_util",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_base/proto:ipopt_return_status_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/piecewise_jerk_speed_nonlinear/proto:piecewise_jerk_speed_nonlinear_cc_proto",
        "@ipopt",
    ],
)

apollo_package()

cpplint()
