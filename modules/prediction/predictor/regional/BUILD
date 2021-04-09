load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "regional_predictor",
    srcs = ["regional_predictor.cc"],
    hdrs = ["regional_predictor.h"],
    deps = [
        "//modules/common/math:kalman_filter",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
        "@glog//:glog",
    ],
)

cc_test(
    name = "regional_predictor_test",
    size = "small",
    srcs = [
        "regional_predictor_test.cc",
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
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/regional:regional_predictor",
        "//modules/prediction/proto:prediction_proto",
        "@gtest//:main",
    ],
)

cpplint()
