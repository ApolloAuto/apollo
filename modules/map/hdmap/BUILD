load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hdmap",
    srcs = [
        "hdmap.cc",
        "hdmap_common.cc",
        "hdmap_impl.cc",
    ],
    hdrs = [
        "hdmap.h",
        "hdmap_common.h",
        "hdmap_impl.h",
    ],
    deps = [
        "//modules/common:macro",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "@glog//:glog",
    ],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "test-data/*",
    ]),
)

cc_test(
    name = "hdmap_map_test",
    size = "medium",
    timeout = "short",
    srcs = [
        "hdmap_common_test.cc",
        "hdmap_impl_test.cc",
    ],
    data = [
        ":testdata",
    ],
    deps = [
        ":hdmap",
        "@glog//:glog",
        "@gtest//:main",
    ],
)

cpplint()
