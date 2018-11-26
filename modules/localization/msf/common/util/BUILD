load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_util_compression",
    srcs = ["compression.cc"],
    hdrs = ["compression.h"],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "localization_msf_common_util_system_utility",
    srcs = ["system_utility.cc"],
    hdrs = ["system_utility.h"],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "localization_msf_common_util_frame_transform",
    srcs = ["frame_transform.cc"],
    hdrs = ["frame_transform.h"],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "localization_msf_common_util",
    hdrs = [
        "extract_ground_plane.h",
        "rect2d.h",
        "math_util.h",
        "voxel_grid_covariance_hdmap.h",
        "time_conversion.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "localization_msf_common_util_compression_test",
    srcs = ["compression_test.cc"],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "@gtest//:main"
    ]
)

cc_test(
    name = "localization_msf_common_util_rect2d_test",
    srcs = ["rect2d_test.cc"],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@gtest//:main"
    ]
)

filegroup(
    name = "localization_msf_common_test_data",
    srcs = glob(["common/test_data/**"]),
)

cc_test(
    name = "localization_msf_common_system_utility_test",
    size = "medium",
    timeout = "short",
    srcs = ["system_utility_test.cc"],
    data = ["localization_msf_common_test_data"],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util_system_utility",
        "@gtest//:main"
    ]
)

cpplint()
