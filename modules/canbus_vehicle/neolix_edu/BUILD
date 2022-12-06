load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "neolix_edu_vehicle_factory",
    srcs = [
        "neolix_edu_vehicle_factory.cc",
    ],
    hdrs = [
        "neolix_edu_vehicle_factory.h",
    ],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":neolix_edu_controller",
        ":neolix_edu_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "libneolix_edu_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":neolix_edu_vehicle_factory"],
)

cc_library(
    name = "neolix_edu_message_manager",
    srcs = [
        "neolix_edu_message_manager.cc",
    ],
    hdrs = [
        "neolix_edu_message_manager.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/neolix_edu/proto:neolix_edu_cc_proto",
        "//modules/canbus_vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "neolix_edu_controller",
    srcs = [
        "neolix_edu_controller.cc",
    ],
    hdrs = [
        "neolix_edu_controller.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        ":neolix_edu_message_manager",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "neolix_edu_controller_test",
    size = "small",
    srcs = ["neolix_edu_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":neolix_edu_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "neolix_edu_message_manager_test",
    size = "small",
    srcs = ["neolix_edu_message_manager_test.cc"],
    deps = [
        ":neolix_edu_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "neolix_edu_vehicle_factory_test",
    size = "small",
    srcs = ["neolix_edu_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":neolix_edu_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-neolix-edu/lib",
    data_dest = "canbus-vehicle-neolix-edu",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":canbus-vehicle-neolix-edu.BUILD",
    ],
    targets = [
        ":libneolix_edu_vehicle_factory_lib.so",
    ],
    deps = [
        ":pb_neolix_edu",
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "canbus-vehicle-neolix-edu/include",
    data = [
        "//modules/canbus_vehicle/neolix_edu/proto:neolix_edu_cc_proto",
    ],
)

install_files(
    name = "pb_neolix_edu",
    dest = "canbus-vehicle-neolix-edu",
    files = [
        "//modules/canbus_vehicle/neolix_edu/proto:neolix_edu_py_pb2",
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
        ":install_neolix_edu_src",
        ":install_neolix_edu_hdrs"
    ],
)

install_src_files(
    name = "install_neolix_edu_src",
    src_dir = ["."],
    dest = "canbus-vehicle-neolix-edu/src",
    filter = "*",
)

install_src_files(
    name = "install_neolix_edu_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-neolix-edu/include",
    filter = "*.h",
)

cpplint()
