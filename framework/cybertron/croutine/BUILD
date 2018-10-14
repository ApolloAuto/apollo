load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "croutine_lib",
    srcs = glob([
        "croutine.cpp",
        "routine_context.cpp",
        "*.S",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron:init_lib",
          "//cybertron/common:common_lib",
          "//cybertron/event:event_lib",
          "//cybertron/time:time_lib",
    ],
)

cc_test(
    name = "croutine_test",
    size = "small",
    srcs = [
        "croutine_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
