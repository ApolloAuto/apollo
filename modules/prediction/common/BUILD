load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_gflags",
    srcs = ["prediction_gflags.cc"],
    hdrs = ["prediction_gflags.h"],
    deps = [
        "//external:gflags",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_library(
    name = "prediction_util",
    srcs = ["prediction_util.cc"],
    hdrs = ["prediction_util.h"],
    deps = [
        "//modules/common/proto:pnc_point_proto",
    ],
)

cc_test(
    name = "prediction_util_test",
    size = "small",
    srcs = ["prediction_util_test.cc"],
    deps = [
        "//modules/prediction/common:prediction_util",
        "@gtest//:main",
    ]
)

cc_library(
    name = "prediction_map",
    srcs = ["prediction_map.cc"],
    hdrs = ["prediction_map.h"],
    deps = [
        "//modules/common:macro",
        "//modules/common/math:linear_interpolation",
        "//modules/common/math:vec2d",
        "//modules/map/hdmap",
        "//modules/map/pnc_map",
        "//modules/prediction/common:prediction_gflags",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "prediction_map_test",
    size = "small",
    srcs = ["prediction_map_test.cc"],
     data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:prediction_gflags",
        "@gtest//:main",
    ]
)

cc_library(
    name = "road_graph",
    srcs = ["road_graph.cc"],
    hdrs = ["road_graph.h"],
    deps = [
        "//modules/common/status",
        "//modules/map/hdmap",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cc_test(
    name = "road_graph_test",
    size = "small",
    srcs = ["road_graph_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:road_graph",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "@gtest//:main",
    ]
)

cpplint()
