load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_base",
    hdrs = [
        "arena_queue.h",
        "atomic_hash_map.h",
        "atomic_rw_lock.h",
        "bounded_queue.h",
        "concurrent_object_pool.h",
        "for_each.h",
        "macros.h",
        "object_pool.h",
        "pthread_rw_lock.h",
        "reentrant_rw_lock.h",
        "rw_lock_guard.h",
        "signal.h",
        "thread_pool.h",
        "thread_safe_queue.h",
        "unbounded_queue.h",
        "wait_strategy.h",
    ],
)

apollo_cc_test(
    name = "atomic_hash_map_test",
    size = "small",
    srcs = ["atomic_hash_map_test.cc"],
    deps = [
        ":cyber_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "atomic_rw_lock_test",
    size = "small",
    srcs = ["atomic_rw_lock_test.cc"],
    deps = [
        ":cyber_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "bounded_queue_test",
    size = "small",
    srcs = ["bounded_queue_test.cc"],
    deps = [
        ":cyber_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "for_each_test",
    size = "small",
    srcs = ["for_each_test.cc"],
    deps = [
        "cyber_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "object_pool_test",
    size = "small",
    srcs = ["object_pool_test.cc"],
    linkopts = [
        "-latomic",
    ],
    deps = [
        ":cyber_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "signal_test",
    size = "small",
    srcs = ["signal_test.cc"],
    deps = [
        ":cyber_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "unbounded_queue_test",
    size = "small",
    srcs = ["unbounded_queue_test.cc"],
    deps = [
        ":cyber_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
