load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_lib",
    srcs = glob([
          "file.cpp",
          "global_data.cpp",
    ]),
    hdrs = glob([
          "*.h",
    ]),
    deps = [
        "//cybertron:init_lib",
        "//cybertron/proto:cyber_config_cc_proto",
        "//cybertron/base:base_lib",
    ],
)

cc_test(
    name = "file_test",
    size = "small",
    srcs = [
        "file_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "logfileobject_test",
    size = "small",
    srcs = [
        "logfileobject_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "log_test",
    size = "small",
    srcs = [
        "log_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
