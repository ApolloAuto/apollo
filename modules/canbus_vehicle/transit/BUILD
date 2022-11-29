load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "transit_vehicle_factory",
    srcs = ["transit_vehicle_factory.cc"],
    hdrs = ["transit_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":transit_controller",
        ":transit_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "libtransit_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":transit_vehicle_factory"],
)

cc_library(
    name = "transit_message_manager",
    srcs = ["transit_message_manager.cc"],
    hdrs = ["transit_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/transit/proto:transit_cc_proto",
        "//modules/canbus_vehicle/transit/protocol:canbus_transit_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "transit_controller",
    srcs = ["transit_controller.cc"],
    hdrs = ["transit_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":transit_message_manager",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/common/kv_db",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "transit_vehicle_factory_test",
    size = "small",
    srcs = ["transit_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":transit_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "transit_controller_test",
    size = "small",
    srcs = ["transit_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":transit_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-transit/lib",
    data_dest = "canbus-vehicle-transit",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-transit.BUILD",
    ],
    targets = [
        ":libtransit_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_transit",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-transit/include",
    data = [
        "//modules/canbus_vehicle/transit/proto:transit_cc_proto",
    ],
)

install_files(
    name = "pb_transit",
    dest = "canbus-vehicle-transit",
    files = [
        "//modules/canbus_vehicle/transit/proto:transit_py_pb2",
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
        ":install_transit_src",
        ":install_transit_hdrs"
    ],
)

install_src_files(
    name = "install_transit_src",
    src_dir = ["."],
    dest = "canbus-vehicle-transit/src",
    filter = "*",
)

install_src_files(
    name = "install_transit_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-transit/include",
    filter = "*.h",
)

cpplint()
