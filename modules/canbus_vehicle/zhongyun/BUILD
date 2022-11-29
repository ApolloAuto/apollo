load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "zhongyun_vehicle_factory",
    srcs = ["zhongyun_vehicle_factory.cc"],
    hdrs = ["zhongyun_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":zhongyun_controller",
        ":zhongyun_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "libzhongyun_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":zhongyun_vehicle_factory"],
)

cc_library(
    name = "zhongyun_message_manager",
    srcs = ["zhongyun_message_manager.cc"],
    hdrs = ["zhongyun_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/zhongyun/proto:zhongyun_cc_proto",
        "//modules/canbus_vehicle/zhongyun/protocol:canbus_zhongyun_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "zhongyun_controller",
    srcs = ["zhongyun_controller.cc"],
    hdrs = ["zhongyun_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":zhongyun_message_manager",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/zhongyun/protocol:canbus_zhongyun_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "zhongyun_message_manager_test",
    size = "small",
    srcs = ["zhongyun_message_manager_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":zhongyun_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "zhongyun_vehicle_factory_test",
    size = "small",
    srcs = ["zhongyun_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":zhongyun_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "zhongyun_controller_test",
    size = "small",
    srcs = ["zhongyun_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":zhongyun_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-zhongyun/lib",
    data_dest = "canbus-vehicle-zhongyun",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-zhongyun.BUILD",
    ],
    targets = [
        ":libzhongyun_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_zhongyun",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-zhongyun/include",
    data = [
        "//modules/canbus_vehicle/zhongyun/proto:zhongyun_cc_proto",
    ],
)

install_files(
    name = "pb_zhongyun",
    dest = "canbus-vehicle-zhongyun",
    files = [
        "//modules/canbus_vehicle/zhongyun/proto:zhongyun_py_pb2",
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
        ":install_zhongyun_src",
        ":install_zhongyun_hdrs"
    ],
)

install_src_files(
    name = "install_zhongyun_src",
    src_dir = ["."],
    dest = "canbus-vehicle-zhongyun/src",
    filter = "*",
)

install_src_files(
    name = "install_zhongyun_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-zhongyun/include",
    filter = "*.h",
)

cpplint()
