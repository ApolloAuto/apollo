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
        "hdmap_util.h",
    ],
    copts = ["-DMODULE_NAME=\\\"map\\\""],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/math:linear_interpolation",
        "//modules/common/util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "@glog",
    ],
)

cc_library(
    name = "hdmap_util",
    srcs = ["hdmap_util.cc"],
    hdrs = ["hdmap_util.h"],
    copts = ["-DMODULE_NAME=\\\"map\\\""],
    deps = [
        ":hdmap",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/util:string_util",
        "//modules/map/relative_map/proto:navigation_proto",
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
    size = "small",
    timeout = "short",
    srcs = [
        "hdmap_common_test.cc",
        "hdmap_impl_test.cc",
    ],
    data = [
        ":testdata",
        "//modules/common/util",
    ],
    deps = [
        ":hdmap",
        "@glog",
        "@gtest//:main",
    ],
)

cc_test(
    name = "hdmap_util_test",
    size = "small",
    timeout = "short",
    srcs = [
        "hdmap_util_test.cc",
    ],
    data = [
        ":testdata",
    ],
    deps = [
        ":hdmap_util",
        "@glog",
        "@gtest//:main",
    ],
)

cpplint()
