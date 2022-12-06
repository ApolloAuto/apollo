load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "canbus_component_lib",
    srcs = ["canbus_component.cc"],
    hdrs = ["canbus_component.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_gflags",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus:sensor_canbus_lib",
        "//modules/common_msgs/guardian_msgs:guardian_cc_proto",
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
    linkstatic = True,
    deps = [
        ":canbus_component_lib",
    ],
)

install(
    name = "testdata",
    data_dest = "canbus/addition_data",
    data = [
        ":test_data",
    ],
)

install(
    name = "install",
    library_dest = "canbus/lib",
    data_dest = "canbus",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus.BUILD",
    ],
    targets = [
        ":libcanbus_component.so",
    ],
    deps = [
        ":pb_canbus",
        ":pb_hdrs",
        ":testdata",
        "//modules/canbus/common:install",
        "//modules/canbus/tools:install",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus/include",
    data = [
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
    ],
)

install_files(
    name = "pb_canbus",
    dest = "canbus",
    files = [
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
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

install_src_files(
    name = "install_src",
    deps = [
        ":install_canbus_src",
        ":install_canbus_hdrs"
    ],
)

install_src_files(
    name = "install_canbus_src",
    src_dir = ["."],
    dest = "canbus/src",
    filter = "*",
)

install_src_files(
    name = "install_canbus_hdrs",
    src_dir = ["."],
    dest = "canbus/include",
    filter = "*.h",
)

cpplint()
