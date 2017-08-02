load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacles_container",
    srcs = ["obstacles_container.cc"],
    hdrs = ["obstacles_container.h"],
    deps = [
        "//modules/common/math:math_utils",
        "//modules/common/util:lru_cache",
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
        "//modules/prediction/container/obstacles:obstacles_container",
        "@gtest//:main",
        "//modules/prediction/common:prediction_gflags",
        "//modules/perception/proto:perception_proto",
        "//modules/common/util:util",
    ],
)

cpplint()
