load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_util_compression",
    srcs = ["compression.cc"],
    hdrs = ["compression.h"],
    deps = [
        "//cyber",
        "//third_party:boost",
        "@eigen",
    ],
)

cc_library(
    name = "localization_msf_common_util_file_utility",
    srcs = ["file_utility.cc"],
    hdrs = ["file_utility.h"],
    deps = [
        "//cyber/common:log",
        "//third_party:boost",
        "@com_google_glog//:glog",
        "@eigen",
        "@fastrtps",
    ],
)

cc_library(
    name = "localization_msf_common_util_system_utility",
    srcs = ["system_utility.cc"],
    hdrs = ["system_utility.h"],
    deps = [
        "//cyber",
        "//third_party:boost",
        "@eigen",
    ],
)

cc_library(
    name = "localization_msf_common_util_frame_transform",
    srcs = ["frame_transform.cc"],
    hdrs = ["frame_transform.h"],
    deps = [
        "//cyber",
        "@eigen",
        "@proj4",
    ],
)

cc_library(
    name = "localization_msf_common_util",
    hdrs = [
        "extract_ground_plane.h",
        "math_util.h",
        "rect2d.h",
        "time_conversion.h",
        "voxel_grid_covariance_hdmap.h",
    ],
    deps = [
        "//cyber",
        "//third_party:boost",
        "@eigen",
    ],
)

cc_test(
    name = "localization_msf_common_util_compression_test",
    size = "small",
    timeout = "short",
    srcs = ["compression_test.cc"],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util_compression",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_msf_common_util_rect2d_test",
    size = "small",
    timeout = "short",
    srcs = ["rect2d_test.cc"],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@com_google_googletest//:gtest_main",
    ],
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
        "//third_party:boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_msf_common_frame_transform_test",
    size = "medium",
    timeout = "short",
    srcs = ["frame_transform_test.cc"],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
