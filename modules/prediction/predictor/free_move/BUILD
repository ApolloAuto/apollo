load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "free_move_predictor",
    srcs = ["free_move_predictor.cc"],
    hdrs = ["free_move_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor",
    ],
)

cc_test(
    name = "free_move_predictor_test",
    size = "small",
    srcs = ["free_move_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/predictor/free_move:free_move_predictor",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
