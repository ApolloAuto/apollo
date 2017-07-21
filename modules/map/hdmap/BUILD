load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hdmap",
    srcs = [
        "hdmap_common.cc",
        "hdmap_impl.cc",
        "hdmap.cc",
    ],
    hdrs = [
        "hdmap_common.h",
        "hdmap_impl.h",
        "hdmap.h",
    ],
    deps = [
        "@glog//:glog",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/common:macro",
        "//modules/map/proto:map_proto",
    ],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "test-data/*.v2",
    ]),
)

cc_test(
    name = "hdmap_map_test",
    srcs = [
        "hdmap_common_test.cc",
        "hdmap_impl_test.cc",
    ],
    data = [
        ":testdata",
    ],
    size = "medium",
    timeout = "short",
    deps = [
        ":hdmap",
        "@glog//:glog",
        "@gtest//:main",
    ],
)

cpplint()
