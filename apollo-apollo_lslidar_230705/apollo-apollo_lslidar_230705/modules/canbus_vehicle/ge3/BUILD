load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "ge3_vehicle_factory",
    srcs = ["ge3_vehicle_factory.cc"],
    hdrs = ["ge3_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":ge3_controller",
        ":ge3_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "libge3_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":ge3_vehicle_factory"],
)

cc_test(
    name = "ge3_vehicle_factory_test",
    size = "small",
    srcs = ["ge3_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":ge3_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ge3_message_manager",
    srcs = ["ge3_message_manager.cc"],
    hdrs = ["ge3_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/ge3/proto:ge3_cc_proto",
        "//modules/canbus_vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ge3_message_manager_test",
    size = "small",
    srcs = ["ge3_message_manager_test.cc"],
    deps = [
        ":ge3_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ge3_controller",
    srcs = ["ge3_controller.cc"],
    hdrs = ["ge3_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":ge3_message_manager",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ge3_controller_test",
    size = "small",
    srcs = ["ge3_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":ge3_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-ge3/lib",
    data_dest = "canbus-vehicle-ge3",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-ge3.BUILD",
    ],
    targets = [
        ":libge3_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_ge3",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-ge3/include",
    data = [
        "//modules/canbus_vehicle/ge3/proto:ge3_cc_proto",
    ],
)

install_files(
    name = "pb_ge3",
    dest = "canbus-vehicle-ge3",
    files = [
        "//modules/canbus_vehicle/ge3/proto:ge3_py_pb2",
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
        ":install_ge3_src",
        ":install_ge3_hdrs"
    ],
)

install_src_files(
    name = "install_ge3_src",
    src_dir = ["."],
    dest = "canbus-vehicle-ge3/src",
    filter = "*",
)

install_src_files(
    name = "install_ge3_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-ge3/include",
    filter = "*.h",
)

cpplint()
