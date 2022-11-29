load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "copts_if_esd_can", "if_esd_can")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "drivers/bin",
    targets = [
      ":can_client_tool",
    ],
)

cc_library(
    name = "can_client_factory",
    srcs = ["can_client_factory.cc"],
    hdrs = ["can_client_factory.h"],
    alwayslink = True,
    copts = copts_if_esd_can(),
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client/fake:fake_can_client",
        "//modules/drivers/canbus/can_client/hermes_can:hermes_can_client",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        ] + if_esd_can([
        "//modules/drivers/canbus/can_client/esd:esd_can_client",
    ]),
)

cc_library(
    name = "can_client",
    hdrs = ["can_client.h"],
    alwayslink = True,
    deps = [
        "//modules/common/util",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
    ],
)

cc_binary(
    name = "can_client_tool",
    srcs = ["can_client_tool.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "can_client_factory_test",
    size = "small",
    srcs = ["can_client_factory_test.cc"],
    deps = [
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
