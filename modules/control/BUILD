load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "control_lib",
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/control/common",
        "//modules/control/controller",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:pad_msg_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
        "//modules/control/submodules:lat_lon_controller_submodule_lib",
        "//modules/control/submodules:mpc_controller_submodule_lib",
        "//modules/control/submodules:postprocessor_submodule_lib",
        "//modules/control/submodules:preprocessor_submodule_lib",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "control_component_lib",
    srcs = ["control_component.cc"],
    hdrs = ["control_component.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":control_lib",
        "//modules/control/common:dependency_injector",
    ],
)

cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":control_component_lib"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libcontrol_component.so",
    ],
    deps = [
        ":pb_control",
    ],
)

install_files(
    name = "pb_control",
    dest = "modules/control",
    files = [
        "//modules/control/proto:pad_msg_py_pb2",
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

cc_test(
    name = "control_component_test",
    size = "small",
    srcs = ["control_component_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":control_component_lib",
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

cpplint()
