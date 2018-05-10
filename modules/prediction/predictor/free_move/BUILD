load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "free_move_predictor",
    srcs = ["free_move_predictor.cc"],
    hdrs = ["free_move_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/math:geometry",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
        "//modules/prediction/proto:feature_proto",
        "@eigen",
    ],
)

cc_test(
    name = "free_move_predictor_test",
    size = "small",
    srcs = [
        "free_move_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:road_graph",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/free_move:free_move_predictor",
        "//modules/prediction/proto:prediction_proto",
        "@gtest//:main",
    ],
)

cpplint()
