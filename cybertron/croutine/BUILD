load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "croutine",
    srcs = [
        "croutine.cc",
    ],
    hdrs = [
        "croutine.h",
    ],
    deps = [
        "//cybertron:init",
        "//cybertron/base:atomic_hash_map",
        "//cybertron/base:atomic_rw_lock",
        "//cybertron/base:bounded_queue",
        "//cybertron/base:macros",
        "//cybertron/base:wait_strategy",
        "//cybertron/common:common",
        "//cybertron/event:perf_event_cache",
        "//cybertron/time:time",
        "//cybertron/croutine:routine_context",
        "//cybertron/croutine:routine_factory",
        "//cybertron/croutine:swap",
    ],
)

cc_library(
    name = "routine_context",
    srcs = [
        "routine_context.cc",
    ],
    hdrs = [
        "routine_context.h",
    ],
    deps = [
          "//cybertron/common:common",
    ],
)

cc_library(
    name = "routine_factory",
    hdrs = [
        "routine_factory.h",
    ],
    deps = [
          "//cybertron/common:common",
          "//cybertron/event:perf_event_cache",
    ],
)

cc_library(
    name = "swap",
    srcs = [
        "swap.S",
    ],
)

cc_test(
    name = "croutine_test",
    size = "small",
    srcs = [
        "croutine_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
