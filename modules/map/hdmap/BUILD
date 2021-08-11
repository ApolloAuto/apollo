load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]

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
    copts = MAP_COPTS,
    deps = [
        "//cyber/common:file",
        "//cyber/common:macros",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/math:linear_interpolation",
        "//modules/common/util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_cc_proto",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "hdmap_util",
    srcs = ["hdmap_util.cc"],
    hdrs = ["hdmap_util.h"],
    copts = MAP_COPTS,
    deps = [
        ":hdmap",
        "//cyber/common:file",
        "//modules/common/configs:config_gflags",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "@com_google_absl//absl/strings",
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
        "//cyber/common:file",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "hdmap_util_test",
    size = "small",
    timeout = "short",
    srcs = ["hdmap_util_test.cc"],
    data = [
        ":testdata",
    ],
    deps = [
        ":hdmap_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
