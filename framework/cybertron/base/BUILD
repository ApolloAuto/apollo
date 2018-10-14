load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_lib",
    hdrs = glob([
          "*.h",
    ]),
    deps = [
    ],
    copts = ["-Iframework"]
)

cc_test(
    name = "any_test",
    size = "small",
    srcs = [
        "any_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "atomic_hash_map_test",
    size = "small",
    srcs = [
        "atomic_hash_map_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "atomic_rw_lock_test",
    size = "small",
    srcs = [
        "atomic_rw_lock_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "bounded_queue_test",
    size = "small",
    srcs = [
        "bounded_queue_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "object_pool_test",
    size = "small",
    srcs = [
        "object_pool_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "signal_test",
    size = "small",
    srcs = [
        "signal_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
