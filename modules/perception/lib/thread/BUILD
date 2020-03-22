load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "thread",
    srcs = [
        "thread.cc",
        "thread_pool.cc",
        "thread_worker.cc",
    ],
    hdrs = [
        "concurrent_queue.h",
        "mutex.h",
        "thread.h",
        "thread_pool.h",
        "thread_worker.h",
    ],
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "concurrent_queue",
    hdrs = ["concurrent_queue.h"],
)

cc_test(
    name = "concurrent_queue_test",
    size = "small",
    srcs = ["concurrent_queue_test.cc"],
    deps = [
        ":thread",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "thread_pool_test",
    size = "small",
    srcs = ["thread_pool_test.cc"],
    deps = [
        ":thread",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "thread_test",
    size = "small",
    srcs = ["thread_test.cc"],
    deps = [
        ":thread",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "thread_worker_test",
    size = "small",
    srcs = ["thread_worker_test.cc"],
    deps = [
        ":thread",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
