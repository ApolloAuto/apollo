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
    deps = [],
)

cc_library(
    name = "prediction_map",
    hdrs = ["prediction_map.h"],
    srcs = ["prediction_map.cc"],
    deps = [
        "@eigen//:eigen",
        "//modules/map/hdmap:hdmap",
        "//modules/common:macro",
        "//modules/prediction/common:prediction_gflags",
        "//modules/map/pnc_map:pnc_path",
    ],

)

cc_library(
    name = "road_graph",
    srcs = ["road_graph.cc"],
    hdrs = ["road_graph.h"],
    deps = [
        "//modules/prediction/proto:lane_graph_proto",
        "//modules/map/hdmap:hdmap",
        "//modules/common/proto:error_code_proto",
        "//modules/prediction/common:prediction_map",
    ],
)

cpplint()
