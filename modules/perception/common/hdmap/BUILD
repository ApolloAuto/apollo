load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "hdmap_files",
    srcs = glob([
        "conf/**",
    ])
)

cc_library(
    name = "hdmap_input",
    srcs = ["hdmap_input.cc"],
    hdrs = ["hdmap_input.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm/geometry:common",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/thread",
    ],
)

#cc_test(
#    name = "hdmap_input_test",
#    size = "small",
#    srcs = ["hdmap_input_test.cc"],
#    deps = [
#        ":hdmap_input",
#        "@com_google_googletest//:gtest_main",
#        "//modules/perception/common/base:base_type",
#        "//modules/perception/common/base:point_cloud",
#        "//modules/perception/common/algorithm/geometry:common",
#    ],
#)

apollo_package()
cpplint()
