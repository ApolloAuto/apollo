load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "gem_vehicle_factory",
    srcs = ["gem_vehicle_factory.cc"],
    hdrs = ["gem_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":gem_controller",
        ":gem_message_manager",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "libgem_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":gem_vehicle_factory"],
)


cc_test(
    name = "gem_vehicle_factory_test",
    size = "small",
    srcs = ["gem_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":gem_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gem_message_manager",
    srcs = ["gem_message_manager.cc"],
    hdrs = ["gem_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/gem/proto:gem_cc_proto",
        "//modules/canbus_vehicle/gem/protocol:canbus_gem_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gem_message_manager_test",
    size = "small",
    srcs = ["gem_message_manager_test.cc"],
    deps = [
        ":gem_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gem_controller",
    srcs = ["gem_controller.cc"],
    hdrs = ["gem_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":gem_message_manager",
	      "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
	      "//modules/canbus_vehicle/gem/proto:gem_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/gem/protocol:canbus_gem_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gem_controller_test",
    size = "small",
    srcs = ["gem_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":gem_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-gem/lib",
    data_dest = "canbus-vehicle-gem",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-gem.BUILD",
    ],
    targets = [
        ":libgem_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_gem",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-gem/include",
    data = [
        "//modules/canbus_vehicle/gem/proto:gem_cc_proto",
    ],
)

install_files(
    name = "pb_gem",
    dest = "canbus-vehicle-gem",
    files = [
        "//modules/canbus_vehicle/gem/proto:gem_py_pb2",
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
        ":install_gem_src",
        ":install_gem_hdrs"
    ],
)

install_src_files(
    name = "install_gem_src",
    src_dir = ["."],
    dest = "canbus-vehicle-gem/src",
    filter = "*",
)

install_src_files(
    name = "install_gem_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-gem/include",
    filter = "*.h",
)

cpplint()
