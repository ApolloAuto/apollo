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
        "//cyber:init",
        "//cyber/base:atomic_hash_map",
        "//cyber/base:atomic_rw_lock",
        "//cyber/base:bounded_queue",
        "//cyber/base:concurrent_object_pool",
        "//cyber/base:macros",
        "//cyber/base:wait_strategy",
        "//cyber/common:common",
        "//cyber/event:perf_event_cache",
        "//cyber/time:time",
        "//cyber/croutine:routine_context",
        "//cyber/croutine:routine_factory",
        "//cyber/croutine:swap",
    ],
    linkopts = ["-latomic"],
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
          "//cyber/common:common",
    ],
)

cc_library(
    name = "routine_factory",
    hdrs = [
        "routine_factory.h",
    ],
    deps = [
          "//cyber/common:common",
          "//cyber/event:perf_event_cache",
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
        "//cyber",
        "@gtest//:main",
    ],
)

cpplint()
