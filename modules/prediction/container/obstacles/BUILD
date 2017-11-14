load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacles_container",
    srcs = ["obstacles_container.cc"],
    hdrs = ["obstacles_container.h"],
    deps = [
        "//modules/common/math:math_utils",
        "//modules/common/util:lru_cache",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/container/pose:pose_container",
    ],
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/math:kalman_filter",
        "//modules/common/math:math_utils",
        "//modules/common/proto:error_code_proto",
        "//modules/common/util:map_util",
        "//modules/map/hdmap",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:road_graph",
        "//modules/prediction/proto:feature_proto",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "obstacles_container_test",
    size = "small",
    srcs = [
        "obstacles_container_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "@gtest//:main",
    ],
)

cc_test(
    name = "obstacle_test",
    size = "small",
    srcs = [
        "obstacle_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "@gtest//:main",
    ],
)

cpplint()
