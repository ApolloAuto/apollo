load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    deps = [
        "//cybertron/base:any",
        "//cybertron/base:atomic_fifo",
        "//cybertron/base:atomic_hash_map",
        "//cybertron/base:atomic_rw_lock",
        "//cybertron/base:bounded_queue",
        "//cybertron/base:concurrent_object_pool",
        "//cybertron/base:macros",
        "//cybertron/base:object_pool",
        "//cybertron/base:reentrant_rw_lock",
        "//cybertron/base:rw_lock_guard",
        "//cybertron/base:signal",
        "//cybertron/base:thread_pool",
        "//cybertron/base:thread_safe_queue",
        "//cybertron/base:unbounded_queue",
        "//cybertron/base:wait_strategy",
    ],
)

cc_library(
    name = "any",
    hdrs = [
        "any.h",
    ],
)

cc_test(
    name = "any_test",
    size = "small",
    srcs = [
        "any_test.cpp",
    ],
    deps = [
        "//cybertron/base:any",
        "@gtest//:main",
    ],
)

cc_library(
    name = "atomic_fifo",
    hdrs = [
        "atomic_fifo.h",
    ],
)

cc_library(
    name = "atomic_hash_map",
    hdrs = [
        "atomic_hash_map.h",
    ],
)

cc_test(
    name = "atomic_hash_map_test",
    size = "small",
    srcs = [
        "atomic_hash_map_test.cpp",
    ],
    deps = [
        "//cybertron/base:atomic_hash_map",
        "@gtest//:main",
    ],
)


cc_library(
    name = "atomic_rw_lock",
    hdrs = [
        "atomic_rw_lock.h",
    ],
    deps = [
        "//cybertron/base:rw_lock_guard",
    ]
)

cc_test(
    name = "atomic_rw_lock_test",
    size = "small",
    srcs = [
        "atomic_rw_lock_test.cpp",
    ],
    deps = [
        "//cybertron/base:atomic_rw_lock",
        "//cybertron/base:reentrant_rw_lock",
        "@gtest//:main",
    ],
)

cc_library(
    name = "bounded_queue",
    hdrs = [
        "bounded_queue.h",
    ],
    deps = [
        "//cybertron/base:macros",
        "//cybertron/base:wait_strategy",
    ],
)

cc_test(
    name = "bounded_queue_test",
    size = "small",
    srcs = [
        "bounded_queue_test.cpp",
    ],
    deps = [
        "//cybertron/base:bounded_queue",
        "@gtest//:main",
    ],
)

cc_library(
    name = "concurrent_object_pool",
    hdrs = [
        "concurrent_object_pool.h",
    ],
)

cc_library(
    name = "macros",
    hdrs = [
        "macros.h",
    ],
)

cc_library(
    name = "object_pool",
    hdrs = [
        "object_pool.h",
    ],
    deps = [
        "//cybertron/base:macros",
    ],
)

cc_test(
    name = "object_pool_test",
    size = "small",
    srcs = [
        "object_pool_test.cpp",
    ],
    deps = [
        "//cybertron/base:object_pool",
        "@gtest//:main",
    ],
)

cc_library(
    name = "reentrant_rw_lock",
    hdrs = [
        "reentrant_rw_lock.h",
    ],
)

cc_library(
    name = "rw_lock_guard",
    hdrs = [
        "rw_lock_guard.h",
    ],
)

cc_library(
    name = "signal",
    hdrs = [
        "signal.h",
    ],
)

cc_test(
    name = "signal_test",
    size = "small",
    srcs = [
        "signal_test.cpp",
    ],
    deps = [
        "//cybertron/base:signal",
        "@gtest//:main",
    ],
)

cc_library(
    name = "thread_pool",
    hdrs = [
        "thread_pool.h",
    ],
)

cc_library(
    name = "thread_safe_queue",
    hdrs = [
        "thread_safe_queue.h",
    ],
)

cc_library(
    name = "unbounded_queue",
    hdrs = [
        "unbounded_queue.h",
    ],
)

cc_library(
    name = "wait_strategy",
    hdrs = [
        "wait_strategy.h",
    ],
)

cpplint()
