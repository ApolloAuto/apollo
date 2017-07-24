load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacles_container",
    srcs = ["obstacles_container.cc"],
    hdrs = ["obstacles_container.h"],
    deps = [
        "//modules/prediction/container:container",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/common/util:lru_cache",
        "//modules/common/math:math_utils",
        "//modules/prediction/container/pose:pose_container",
    ],
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    deps = [
        "@eigen//:eigen",
        "//modules/common:log",
        "//modules/common/math:kalman_filter",
        "//modules/common/proto:error_code_proto",
        "//modules/common/math:math_utils",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/proto:feature_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:road_graph",
        "//modules/map/hdmap:hdmap",
    ],
)

cpplint()
