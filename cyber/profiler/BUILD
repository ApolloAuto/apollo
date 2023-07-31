load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/platform:build_defs.bzl", "if_profiler")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "block_manager",
    srcs = [
        "block_manager.cc",
        "block.cc",
        "frame.cc",
    ],
    hdrs = [
        "block_manager.h",
        "block.h",
        "frame.h",
    ],
    deps = [
        "//cyber/common",
        "//cyber/croutine",
    ],
)

cc_library(
    name = "profiler",
    hdrs = ["profiler.h"],
    deps = [
        ":block_manager",
    ],
)

cc_test(
    name = "profiler_test",
    size = "small",
    srcs = ["profiler_test.cc"],
    copts = if_profiler(),
    deps = [
        ":profiler",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
