load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "blocker_lib",
    srcs = glob([
        "blocker_manager.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron:init_lib",
          "//cybertron/time:time",
    ],
)

cc_test(
    name = "blocker_manager_test",
    size = "small",
    srcs = [
        "blocker_manager_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "blocker_test",
    size = "small",
    srcs = [
        "blocker_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cpplint()
