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
    linkopts = ["-latomic"],
    deps = [
        "//cyber/base:atomic_hash_map",
        "//cyber/base:atomic_rw_lock",
        "//cyber/base:bounded_queue",
        "//cyber/base:concurrent_object_pool",
        "//cyber/base:macros",
        "//cyber/base:wait_strategy",
        "//cyber/common",
        "//cyber/croutine:routine_context",
        "//cyber/croutine:routine_factory",
        "//cyber/croutine:swap",
        "//cyber/event:perf_event_cache",
        "//cyber/time",
    ],
)

cc_library(
    name = "routine_context",
    srcs = [
        "detail/routine_context.cc",
    ],
    hdrs = [
        "detail/routine_context.h",
    ],
    deps = [
        "//cyber/common",
    ],
)

cc_library(
    name = "routine_factory",
    hdrs = [
        "routine_factory.h",
    ],
    deps = [
        "//cyber/common",
        "//cyber/event:perf_event_cache",
    ],
)

cc_library(
    name = "swap",
    srcs = select({
        ":x86_mode": [
            "detail/swap_x86_64.S",
        ],
        ":arm_mode": [
            "detail/swap_aarch64.S",
        ],
    }),
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

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)

cpplint()
