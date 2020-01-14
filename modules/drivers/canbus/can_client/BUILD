load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_client_factory",
    srcs = ["can_client_factory.cc"],
    hdrs = ["can_client_factory.h"],
    deps = [
        "//cyber",
        "//modules/common/util:factory",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client/fake:fake_can_client",
        "//modules/drivers/canbus/can_client/hermes_can:hermes_can_client",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/drivers/canbus/proto:canbus_proto",
    ] + select({
        "//tools/platforms:use_esd_can": [
            "//modules/drivers/canbus/can_client/esd:esd_can_client",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "can_client",
    hdrs = ["can_client.h"],
    deps = [
        "//modules/common/util",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/drivers/canbus/proto:canbus_proto",
    ],
)

cc_binary(
    name = "can_client_tool",
    srcs = ["can_client_tool.cc"],
    deps = [
        "//modules/common/time",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/proto:canbus_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "can_client_factory_test",
    size = "small",
    srcs = ["can_client_factory_test.cc"],
    deps = [
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/proto:canbus_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
