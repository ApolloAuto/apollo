load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_lexus",
    srcs = [
        "lexus_controller.cc",
        "lexus_message_manager.cc",
        "lexus_vehicle_factory.cc",
    ] + glob(["protocol/*.cc"]),
    hdrs = [
        "lexus_controller.h",
        "lexus_message_manager.h",
        "lexus_vehicle_factory.h",
    ] + glob(["protocol/*.h"]),
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/lexus/proto:lexus_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "liblexus_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_lexus"],
)

apollo_cc_test(
    name = "lexus_controller_test",
    size = "small",
    srcs = ["lexus_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_lexus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "lexus_vehicle_factory_test",
    size = "small",
    srcs = ["lexus_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_lexus",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()

cpplint()