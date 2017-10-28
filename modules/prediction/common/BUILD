load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_gflags",
    srcs = ["prediction_gflags.cc"],
    hdrs = ["prediction_gflags.h"],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "prediction_util",
    srcs = ["prediction_util.cc"],
    hdrs = ["prediction_util.h"],
    deps = [
        ":prediction_gflags",
        ":prediction_map",
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/proto:lane_graph_proto",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "prediction_util_test",
    size = "small",
    srcs = ["prediction_util_test.cc"],
    deps = [
        ":prediction_util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "prediction_map",
    srcs = ["prediction_map.cc"],
    hdrs = ["prediction_map.h"],
    deps = [
        ":prediction_gflags",
        "//modules/common:macro",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:linear_interpolation",
        "//modules/common/math:vec2d",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "prediction_map_test",
    size = "small",
    srcs = ["prediction_map_test.cc"],
    data = [
        "//modules/common/configs:config_gflags",
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":kml_map_based_test",
        ":prediction_gflags",
        ":prediction_map",
        "@gtest//:main",
    ],
)

cc_library(
    name = "road_graph",
    srcs = ["road_graph.cc"],
    hdrs = ["road_graph.h"],
    deps = [
        ":prediction_map",
        "//modules/common/status",
        "//modules/map/hdmap",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cc_test(
    name = "road_graph_test",
    size = "small",
    srcs = ["road_graph_test.cc"],
    data = [
        "//modules/common/configs:config_gflags",
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":kml_map_based_test",
        ":prediction_gflags",
        ":prediction_map",
        ":road_graph",
        "@gtest//:main",
    ],
)

cc_library(
    name = "kml_map_based_test",
    hdrs = ["kml_map_based_test.h"],
    deps = [
        "//modules/common/configs:config_gflags",
        "@gtest//:gtest",
    ],
)

cpplint()
