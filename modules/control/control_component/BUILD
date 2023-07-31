load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "control_lib",
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/control/control_component/proto:preprocessor_cc_proto",
        "//modules/control/control_component/proto:pipeline_cc_proto",
        "//modules/control/control_component/submodules:lat_lon_controller_submodule_lib",
        "//modules/control/control_component/submodules:mpc_controller_submodule_lib",
        "//modules/control/control_component/submodules:postprocessor_submodule_lib",
        "//modules/control/control_component/submodules:preprocessor_submodule_lib",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True
)

cc_library(
    name = "control_component_lib",
    srcs = ["control_component.cc"],
    hdrs = ["control_component.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":control_lib",
        "//modules/control/controllers/control_task_base/common:dependency_injector",
        "//modules/control/controllers/control_task_base:control_task_agent",
    ],
    alwayslink = True
)

cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":control_component_lib",

    ],
)

# install(
#     name = "testdata",
#     data_dest = "control/addition_data",
#     data = [
#         ":test_data",
#     ],
# )

# install(
#     name = "install",
#     library_dest = "control/lib",
#     data_dest = "control",
#     data = [
#         ":runtime_data",
#         ":cyberfile.xml",
#         "control.BUILD",
#     ],
#     targets = [
#         ":libcontrol_component.so",
#     ],
#     deps = [
#         ":pb_control",
#         ":pb_hdrs",
#         ":testdata",
#         "//modules/control/submodules:install",
#         "//modules/control/tools:install",
#         "//modules/control/proto:py_pb_control",
#         "//modules/control/common:install"
#     ],
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "control/include",
#     data = [
#         "//modules/control/proto:calibration_table_cc_proto",
#         "//modules/control/proto:control_common_conf_cc_proto",
#         "//modules/control/proto:control_conf_cc_proto",
#         "//modules/control/proto:gain_scheduler_conf_cc_proto",
#         "//modules/control/proto:lat_controller_conf_cc_proto",
#         "//modules/control/proto:leadlag_conf_cc_proto",
#         "//modules/control/proto:local_view_cc_proto",
#         "//modules/control/proto:lon_controller_conf_cc_proto",
#         "//modules/control/proto:mpc_controller_conf_cc_proto",
#         "//modules/control/proto:mrac_conf_cc_proto",
#         "//modules/control/proto:pid_conf_cc_proto",
#         "//modules/control/proto:preprocessor_cc_proto",       
#     ],
# )

# install_files(
#     name = "pb_control",
#     dest = "control",
#     files = [
#         "//modules/common_msgs/control_msgs:control_pad_msg_py_pb2",
#     ],
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_control_src",
#         ":install_control_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_control_src",
#     src_dir = ["."],
#     dest = "control/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_control_hdrs",
#     src_dir = ["."],
#     dest = "control/include",
#     filter = "*.h",
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_test(
    name = "control_component_test",
    size = "small",
    srcs = ["control_component_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":control_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

# TODO(storypku): split test_data
filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()
cpplint()
