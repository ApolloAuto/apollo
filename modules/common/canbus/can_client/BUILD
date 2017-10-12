load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_client_factory",
    srcs = [
        "can_client_factory.cc",
    ],
    hdrs = [
        "can_client_factory.h",
    ],
    deps = [
        "//modules/common/canbus/can_client",
        "//modules/common/canbus/common:canbus_common",
        "//modules/common/canbus/proto:canbus_proto",
        "//modules/common/util:factory",
        "//modules/common/canbus/can_client/fake:fake_can_client",
    ] + select({
        "//tools/platforms:use_esd_can": [
            "//modules/common/canbus/can_client/esd:esd_can_client",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "can_client",
    hdrs = [
        "can_client.h",
    ],
    deps = [
        "//modules/common/canbus/common:canbus_common",
        "//modules/common/canbus/proto:canbus_proto",
        "//modules/common/util",
    ],
)

cc_binary(
    name = "can_client_tool",
    srcs = ["can_client_tool.cc"],
    deps = [
        "//external:gflags",
        "//modules/common/canbus/can_client:can_client_factory",
        "//modules/common/canbus/proto:canbus_proto",
        "//modules/common",
    ],
)

cc_test(
    name = "can_client_factory_test",
    size = "small",
    srcs = ["can_client_factory_test.cc"],
    deps = [
        "//modules/common/canbus/can_client:can_client_factory",
        "//modules/common/canbus/proto:canbus_proto",
        "@gtest//:main",
    ],
)

cpplint()
