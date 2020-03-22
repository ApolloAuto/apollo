load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "chi_squared_cdf",
    hdrs = [
        "chi_squared_cdf_1_0.0500_0.999900.h",
        "chi_squared_cdf_2_0.0500_0.999900.h",
    ],
)

cc_library(
    name = "hm_tracks_objects_match",
    srcs = ["hm_tracks_objects_match.cc"],
    hdrs = ["hm_tracks_objects_match.h"],
    deps = [
        ":track_object_distance",
        "//modules/perception/common/graph:gated_hungarian_bigraph_matcher",
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/lib/interface",
    ],
)

cc_library(
    name = "probabilities",
    srcs = ["probabilities.cc"],
    hdrs = ["probabilities.h"],
)

cc_library(
    name = "projection_cache",
    hdrs = ["projection_cache.h"],
)

cc_library(
    name = "track_object_distance",
    srcs = ["track_object_distance.cc"],
    hdrs = ["track_object_distance.h"],
    deps = [
        ":chi_squared_cdf",
        ":probabilities",
        ":projection_cache",
        ":track_object_similarity",
        "//modules/perception/base:base_type",
        "//modules/perception/base:camera",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/geometry:camera_homography",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:fusion_log",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/base:track",
        "//modules/perception/fusion/common:camera_util",
    ],
)

cc_library(
    name = "track_object_similarity",
    srcs = ["track_object_similarity.cc"],
    hdrs = ["track_object_similarity.h"],
    deps = [
        ":chi_squared_cdf",
        ":probabilities",
        ":projection_cache",
        "//modules/perception/fusion/base:fusion_log",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/fusion/common:camera_util",
    ],
)

cc_test(
    name = "hm_data_association_test",
    size = "small",
    srcs = ["hm_data_association_test.cc"],
    deps = [
        ":probabilities",
        ":projection_cache",
        ":track_object_distance",
        ":track_object_similarity",
        "//modules/perception/base:frame",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/fusion/base:track",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "track_object_distance_test",
    size = "small",
    srcs = ["track_object_distance_test.cc"],
    deps = [
        ":projection_cache",
        ":track_object_distance",
        ":track_object_similarity",
        "//modules/perception/base:frame",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/fusion/base:track",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "track_object_test",
    size = "small",
    srcs = ["track_object_test.cc"],
    deps = [
        ":hm_tracks_objects_match",
        "//modules/perception/base:frame",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/fusion/base:track",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
