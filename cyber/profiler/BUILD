load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_profiler",
    hdrs = [
        "profiler.h",
        "block_manager.h",
        "block.h",
        "frame.h",
    ],
    srcs = [
        "block_manager.cc",
        "block.cc",
        "frame.cc",
    ],
    deps = [
        "//cyber/common:cyber_common",
        "//cyber/croutine:cyber_croutine",
    ],
)

apollo_cc_test(
    name = "profiler_test",
    size = "small",
    srcs = ["profiler_test.cc"],
    deps = [
        ":cyber_profiler",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
