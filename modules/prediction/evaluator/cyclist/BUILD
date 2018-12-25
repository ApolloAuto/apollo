load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cyclist_keep_lane_evaluator",
    srcs = [
        "cyclist_keep_lane_evaluator.cc",
    ],
    hdrs = [
        "cyclist_keep_lane_evaluator.h",
    ],
    deps = [
        "//modules/common/math:geometry",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:validation_checker",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cc_test(
    name = "cyclist_keep_lane_evaluator_test",
    size = "small",
    srcs = [
        "cyclist_keep_lane_evaluator_test.cc",
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
        ":cyclist_keep_lane_evaluator",
        "@gtest//:main",
    ],
)

cpplint()
