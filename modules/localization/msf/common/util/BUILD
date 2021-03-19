load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "compression",
    srcs = ["compression.cc"],
    hdrs = ["compression.h"],
    deps = [
        "//cyber",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "file_utility",
    srcs = ["file_utility.cc"],
    hdrs = ["file_utility.h"],
    deps = [
        "//cyber/common:log",
        "@boost",
        "@com_github_google_glog//:glog",
        "@eigen",
        "@fastrtps",
    ],
)

cc_library(
    name = "system_utility",
    srcs = ["system_utility.cc"],
    hdrs = ["system_utility.h"],
    deps = [
        "//cyber",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "frame_transform",
    srcs = ["frame_transform.cc"],
    hdrs = ["frame_transform.h"],
    deps = [
        "//cyber",
        "@com_google_absl//absl/strings",
        "@eigen",
        "@proj",
    ],
)

cc_library(
    name = "util",
    hdrs = [
        "extract_ground_plane.h",
        "math_util.h",
        "rect2d.h",
        "time_conversion.h",
        "voxel_grid_covariance_hdmap.h",
    ],
    deps = [
        "//cyber",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "base_map_cache",
    hdrs = [
        "base_map_cache.h",
    ],
    deps = [
        "//modules/common/util:lru_cache",
    ],
)

cc_test(
    name = "compression_test",
    size = "small",
    timeout = "short",
    srcs = ["compression_test.cc"],
    deps = [
        "//modules/localization/msf/common/util:compression",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "rect2d_test",
    size = "small",
    timeout = "short",
    srcs = ["rect2d_test.cc"],
    deps = [
        "//modules/localization/msf/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob(["common/test_data/**"]),
)

cc_test(
    name = "system_utility_test",
    size = "medium",
    timeout = "short",
    srcs = ["system_utility_test.cc"],
    data = [":test_data"],
    deps = [
        "//modules/localization/msf/common/util:system_utility",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "frame_transform_test",
    size = "medium",
    timeout = "short",
    srcs = ["frame_transform_test.cc"],
    deps = [
        "//modules/localization/msf/common/util:frame_transform",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "base_map_cache_test",
    srcs = ["base_map_cache_test.cc"],
    deps = [
        ":base_map_cache",
        "//cyber/common",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
