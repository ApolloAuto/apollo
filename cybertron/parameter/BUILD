load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parameter_lib",
    srcs = glob([
        "parameter_client.cpp",
        "parameter.cpp",
        "parameter_server.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/node:node_lib",
        "//cybertron/data:data_lib",
        "//cybertron/message:message_lib",
        "//cybertron/service:service_lib",
        "//cybertron/proto:parameter_cc_proto",
        "@fastrtps",
    ],
)

cc_test(
    name = "parameter_client_test",
    size = "small",
    srcs = [
        "parameter_client_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "parameter_server_test",
    size = "small",
    srcs = [
        "parameter_server_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "parameter_test",
    size = "small",
    srcs = [
        "parameter_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
