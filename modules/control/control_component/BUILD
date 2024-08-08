load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_cc_library(
    name = "apollo_control_lib",
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/util:common_util",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:control_interactive_msg_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/control/control_component/proto:pipeline_cc_proto",
        "//modules/control/control_component/proto:preprocessor_cc_proto",
        "//modules/control/control_component/submodules:lat_lon_controller_submodule_lib",
        "//modules/control/control_component/submodules:mpc_controller_submodule_lib",
        "//modules/control/control_component/submodules:postprocessor_submodule_lib",
        "//modules/control/control_component/submodules:preprocessor_submodule_lib",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_component(
    name = "libcontrol_component.so",
    srcs = ["control_component.cc"],
    hdrs = ["control_component.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":apollo_control_lib",
        "//modules/control/control_component/controller_task_base:control_task_agent",
        "//modules/control/control_component/controller_task_base/common:dependency_injector",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_cc_test(
    name = "control_component_test",
    size = "small",
    srcs = ["control_component_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    linkstatic = True,
    deps = [
        ":DO_NOT_IMPORT_control_component",
        "@com_google_googletest//:gtest_main",
    ],
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
