load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "ch_vehicle_factory",
    srcs = ["ch_vehicle_factory.cc"],
    hdrs = ["ch_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":ch_controller",
        ":ch_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "libch_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":ch_vehicle_factory"],
)

cc_library(
    name = "ch_message_manager",
    srcs = ["ch_message_manager.cc"],
    hdrs = ["ch_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/ch/proto:ch_cc_proto",
        "//modules/canbus_vehicle/ch/protocol:canbus_ch_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "ch_controller",
    srcs = ["ch_controller.cc"],
    hdrs = ["ch_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":ch_message_manager",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/ch/protocol:canbus_ch_protocol",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

cc_test(
    name = "ch_controller_test",
    size = "small",
    srcs = ["ch_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":ch_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ch_message_manager_test",
    size = "small",
    srcs = ["ch_message_manager_test.cc"],
    deps = [
        ":ch_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ch_vehicle_factory_test",
    size = "small",
    srcs = ["ch_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":ch_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-ch/lib",
    data_dest = "canbus-vehicle-ch",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-ch.BUILD",
    ],
    targets = [
        ":libch_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_ch",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-ch/include",
    data = [
        "//modules/canbus_vehicle/ch/proto:ch_cc_proto",
    ],
)

install_files(
    name = "pb_ch",
    dest = "canbus-vehicle-ch",
    files = [
        "//modules/canbus_vehicle/ch/proto:ch_py_pb2",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_ch_src",
        ":install_ch_hdrs"
    ],
)

install_src_files(
    name = "install_ch_src",
    src_dir = ["."],
    dest = "canbus-vehicle-ch/src",
    filter = "*",
)

install_src_files(
    name = "install_ch_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-ch/include",
    filter = "*.h",
)

cpplint()
