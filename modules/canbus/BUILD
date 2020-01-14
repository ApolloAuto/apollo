load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_lib",
    srcs = ["canbus_component.cc"],
    hdrs = ["canbus_component.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/guardian/proto:guardian_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "canbus_test",
    size = "small",
    srcs = ["canbus_test.cc"],
    deps = [
        ":canbus_lib",
        "//modules/common/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libcanbus.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":canbus_lib",
    ],
)

filegroup(
    name = "canbus_testdata",
    srcs = glob(["testdata/**"]),
)

cpplint()
