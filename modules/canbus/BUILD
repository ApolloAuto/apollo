load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "canbus_component_lib",
    srcs = ["canbus_component.cc"],
    hdrs = ["canbus_component.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/guardian/proto:guardian_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "canbus_test",
    size = "small",
    srcs = ["canbus_test.cc"],
    deps = [
        ":canbus_component_lib",
        "//modules/common/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libcanbus_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":canbus_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libcanbus_component.so",
    ],
    deps = [
        ":pb_canbus",
        "//cyber:install",
    ],
)

install_files(
    name = "pb_canbus",
    dest = "modules/canbus",
    files = [
        "//modules/canbus/proto:chassis_py_pb2",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

#TODO(storypku):
# split test_data for each vehicle
filegroup(
    name = "test_data",
    srcs = glob(["testdata/**"]),
)

cpplint()
