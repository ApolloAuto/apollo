load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_lib",
    srcs = glob([
    ]),
    hdrs = glob([
        "*.h",
        "**/*.h",
    ]),
    deps = [
        "//cybertron/proto:component_config_cc_proto",
    ],        
)

cc_test(
    name = "cache_buffer_test",
    size = "small",
    srcs = [
        "cache_buffer_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "data_visitor_test",
    size = "small",
    srcs = [
        "data_visitor_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "data_dispatcher_test",
    size = "small",
    srcs = [
        "data_dispatcher_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "channel_buffer_test",
    size = "small",
    srcs = [
        "channel_buffer_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
