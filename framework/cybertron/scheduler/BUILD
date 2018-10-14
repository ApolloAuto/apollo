load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scheduler_lib",
    srcs = glob([
        "processor.cpp",
        "processor_context.cpp",
        "scheduler.cpp",
        "policy/*.cpp",
    ]),
    hdrs = glob([
        "policy/*.h",
        "*.h",
    ]),
    deps = [
          "//cybertron/croutine:croutine_lib",
          "//cybertron/data:data_lib",
    ],
)

cc_test(
    name = "scheduler_test",
    size = "small",
    srcs = [
        "scheduler_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "scheduler_policy_test",
    size = "small",
    srcs = [
        "scheduler_policy_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
