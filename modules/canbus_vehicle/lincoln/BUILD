load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "lincoln_vehicle_factory",
    srcs = ["lincoln_vehicle_factory.cc"],
    hdrs = ["lincoln_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":lincoln_controller",
        ":lincoln_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "liblincoln_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lincoln_vehicle_factory"],
)

cc_library(
    name = "lincoln_message_manager",
    srcs = ["lincoln_message_manager.cc"],
    hdrs = ["lincoln_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/lincoln/proto:lincoln_cc_proto",
        "//modules/canbus_vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "lincoln_controller",
    srcs = ["lincoln_controller.cc"],
    hdrs = ["lincoln_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":lincoln_message_manager",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus_vehicle/lincoln/proto:lincoln_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "//modules/common/kv_db",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lincoln_controller_test",
    size = "small",
    srcs = ["lincoln_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        "//modules/canbus_vehicle/lincoln:lincoln_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lincoln_message_manager_test",
    size = "small",
    srcs = ["lincoln_message_manager_test.cc"],
    deps = [
        "//modules/canbus_vehicle/lincoln:lincoln_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lincoln_vehicle_factory_test",
    size = "small",
    srcs = ["lincoln_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":lincoln_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-lincoln/lib",
    data_dest = "canbus-vehicle-lincoln",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-lincoln.BUILD",
    ],
    targets = [
        ":liblincoln_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_lincoln",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-lincoln/include",
    data = [
        "//modules/canbus_vehicle/lincoln/proto:lincoln_cc_proto",
    ],
)

install_files(
    name = "pb_lincoln",
    dest = "canbus-vehicle-lincoln",
    files = [
        "//modules/canbus_vehicle/lincoln/proto:lincoln_py_pb2",
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
        ":install_lincoln_src",
        ":install_lincoln_hdrs"
    ],
)

install_src_files(
    name = "install_lincoln_src",
    src_dir = ["."],
    dest = "canbus-vehicle-lincoln/src",
    filter = "*",
)

install_src_files(
    name = "install_lincoln_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-lincoln/include",
    filter = "*.h",
)

cpplint()
