package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dp_poly_path",
    srcs = [
        "dp_poly_path_config.cpp",
        "dp_road_graph.cpp",
        "graph_edge.cpp",
        "graph_vertex.cpp",
        "path_sampler.cpp",
        "trajectory_cost.cpp",
    ],
    hdrs = [
        "dp_poly_path_config.h",
        "dp_road_graph.h",
        "graph_edge.h",
        "graph_vertex.h",
        "path_sampler.h",
        "trajectory_cost.h",
    ],
    deps = [
        "@eigen//:eigen",
        "//modules/common/math",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:data_center",
        "//modules/planning/common:decision_data",
        "//modules/planning/common:obstacle",
        "//modules/planning/common/path",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed",
        "//modules/planning/math:sl_analytic_transformation",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/reference_line:lib_reference_line",
    ],
)
