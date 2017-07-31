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
        "//modules/common/proto:path_point_proto",
    ],
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
        "//modules/map/pnc_map:path",
        "//modules/prediction/common:prediction_gflags",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "road_graph",
    srcs = ["road_graph.cc"],
    hdrs = ["road_graph.h"],
    deps = [
        "//modules/common/proto:error_code_proto",
        "//modules/map/hdmap",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cpplint()
