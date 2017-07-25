load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dp_poly_path",
    srcs = [
        "dp_road_graph.cc",
        "graph_edge.cc",
        "graph_vertex.cc",
        "path_sampler.cc",
        "trajectory_cost.cc",
    ],
    hdrs = [
        "dp_road_graph.h",
        "graph_edge.h",
        "graph_vertex.h",
        "path_sampler.h",
        "trajectory_cost.h",
    ],
    deps = [
        "@eigen//:eigen",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:decision_data",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/path",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/math:sl_analytic_transformation",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/proto:dp_poly_path_config_proto",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "dp_poly_path_optimizer",
    srcs = [
        "dp_poly_path_optimizer.cc",
    ],
    hdrs = [
        "dp_poly_path_optimizer.h",
    ],
    deps = [
        ":dp_poly_path",
        "//modules/planning/optimizer:path_optimizer",
    ],
)

cpplint()
