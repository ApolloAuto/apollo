load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "devkit_vehicle_factory",
    srcs = [
        "devkit_vehicle_factory.cc",
    ],
    hdrs = [
        "devkit_vehicle_factory.h",
    ],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":devkit_controller",
        ":devkit_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "libdevkit_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":devkit_vehicle_factory"],
)

cc_test(
    name = "devkit_vehicle_factory_test",
    size = "small",
    srcs = ["devkit_vehicle_factory_test.cc"],
    linkstatic = True,
    deps = [
        ":devkit_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "devkit_message_manager",
    srcs = ["devkit_message_manager.cc"],
    hdrs = ["devkit_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/devkit/proto:devkit_cc_proto",
        "//modules/canbus_vehicle/devkit/protocol:canbus_devkit_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "devkit_message_manager_test",
    size = "small",
    srcs = ["devkit_message_manager_test.cc"],
    deps = [
        ":devkit_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "devkit_controller",
    srcs = ["devkit_controller.cc"],
    hdrs = ["devkit_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":devkit_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus_vehicle/devkit/proto:devkit_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/devkit/protocol:canbus_devkit_protocol",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

cc_test(
    name = "devkit_controller_test",
    size = "small",
    srcs = ["devkit_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":devkit_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-devkit/lib",
    data_dest = "canbus-vehicle-devkit",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-devkit.BUILD",
    ],
    targets = [
        ":libdevkit_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_devkit",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-devkit/include",
    data = [
        "//modules/canbus_vehicle/devkit/proto:devkit_cc_proto",
    ],
)

install_files(
    name = "pb_devkit",
    dest = "canbus-vehicle-devkit",
    files = [
        "//modules/canbus_vehicle/devkit/proto:devkit_py_pb2",
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
        ":install_devkit_src",
        ":install_devkit_hdrs"
    ],
)

install_src_files(
    name = "install_devkit_src",
    src_dir = ["."],
    dest = "canbus-vehicle-devkit/src",
    filter = "*",
)

install_src_files(
    name = "install_devkit_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-devkit/include",
    filter = "*.h",
)

cpplint()
